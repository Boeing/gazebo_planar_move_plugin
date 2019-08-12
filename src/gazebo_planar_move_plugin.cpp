#include <gazebo_planar_move_plugin/gazebo_planar_move_plugin.h>


#include <boost/bind.hpp>
#include <cmath>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include <string>

#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ros/advertise_options.h>

namespace gazebo
{

namespace
{

template <typename TYPE>
void loadParam(sdf::ElementPtr sdf, TYPE& value, const TYPE& default_value, const std::string& param_name,
               const std::string& robot_namespace)
{
    if (!sdf->HasElement(param_name))
    {
        ROS_WARN_STREAM_NAMED("planar_move", "PlanarMovePlugin (ns = " << robot_namespace << ") missing <" << param_name
                                                                       << ">, defaults to \"" << default_value << "\"");
        value = default_value;
    }
    else
    {
        value = sdf->GetElement(param_name)->Get<TYPE>();
    }
}
}  // namespace

PlanarMove::PlanarMove() : publish_odometry_(false), publish_tf_(false), x_(0), y_(0), rot_(0), gz_time_last_(0)

{
}

PlanarMove::~PlanarMove()
{
}

// cppcheck-suppress unusedFunction
void PlanarMove::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    parent_ = parent;

    loadParam(sdf, robot_namespace_, std::string("/"), std::string("robot_namespace"), robot_namespace_);
    loadParam(sdf, command_topic_, std::string("cmd_vel"), std::string("command_topic"), robot_namespace_);
    loadParam(sdf, odometry_topic_, std::string("odom"), std::string("odometry_topic"), robot_namespace_);
    loadParam(sdf, odometry_frame_, std::string("odom"), std::string("odometry_frame"), robot_namespace_);
    loadParam(sdf, robot_base_frame_, std::string("base_link"), std::string("robot_frame"), robot_namespace_);
    loadParam(sdf, publish_odometry_, true, std::string("publish_odometry"), robot_namespace_);
    loadParam(sdf, publish_tf_, true, std::string("publish_tf"), robot_namespace_);
    loadParam(sdf, publish_imu_, false, std::string("publish_imu"), robot_namespace_);
    loadParam(sdf, control_mode_, std::string("position"), std::string("control_mode"), robot_namespace_);

    x_ = 0;
    y_ = 0;
    rot_ = 0;

    nh_ = ros::NodeHandle(robot_namespace_);

    vel_sub_ = nh_.subscribe(command_topic_, 1, &PlanarMove::cmdVelCallback, this);
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic_, 1);
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu", 1);

    // listen to the update event (broadcast every simulation iteration)
    gz_time_last_ = 0.0;
    update_connection_ = event::Events::ConnectBeforePhysicsUpdate(boost::bind(&PlanarMove::UpdateChild, this));

    links_list_ = parent_->GetLinks();
    base_link_ = parent_->GetLink(robot_base_frame_);
}

void PlanarMove::UpdateChild()
{
    // block any other physics pose updates
    boost::recursive_mutex::scoped_lock plock(*parent_->GetWorld()->Physics()->GetPhysicsUpdateMutex());

    double gz_time_now = parent_->GetWorld()->SimTime().Double();

    const double dt = gz_time_now - gz_time_last_;
    gz_time_last_ = gz_time_now;

    const ros::Time current_time = ros::Time(gz_time_now);

    std::lock_guard<std::mutex> lock(lock_);
    const bool is_paused = parent_->GetWorld()->IsPaused();
    const bool is_physics_enabled = parent_->GetWorld()->PhysicsEnabled();

    parent_->GetWorld()->SetPaused(true);

    // Get the simulation time and period
    ignition::math::Pose3d current_pose = parent_->WorldPose();
    double current_yaw = current_pose.Rot().Yaw();
    ignition::math::Pose3d new_pose;

    //
    // Position control mode
    //
    if (control_mode_ == "position")
    {
        // We need to do this so dx and dy are in world frame since cmd_vel is in robot frame
        new_pose.Pos().X() = current_pose.Pos().X() + dt * x_ * cos(current_yaw);
        new_pose.Pos().Y() = current_pose.Pos().Y() + dt * x_ * sin(current_yaw);
        new_pose.Pos().X() = new_pose.Pos().X() - dt * y_ * cos(M_PI / 2 - current_yaw);
        new_pose.Pos().Y() = new_pose.Pos().Y() + dt * y_ * sin(M_PI / 2 - current_yaw);
        new_pose.Rot().Euler(0.0, 0.0, current_yaw + dt * rot_);

        if (base_link_ == nullptr)
        {
            ROS_FATAL_THROTTLE(1, "Model has no link named 'base link'");
        }
        else
        {
            parent_->SetLinkWorldPose(new_pose, base_link_);

            // Hack disables physics, required after call to any physics related function call
            if (!is_physics_enabled)
            {
                for (physics::LinkPtr link : links_list_)
                {
                    link->SetEnabled(false);
                }
            }
        }
    }

    //
    // Velocity control mode
    //
    else if (control_mode_ == "velocity")
    {
        if (new_cmd_)
        {
            ignition::math::Pose3d pose = parent_->WorldPose();
            float yaw = pose.Rot().Yaw();
            parent_->SetLinearVel(
                ignition::math::Vector3d(x_ * cosf(yaw) - y_ * sinf(yaw), y_ * cosf(yaw) + x_ * sinf(yaw), 0));
            parent_->SetAngularVel(ignition::math::Vector3d(0, 0, rot_));
            new_cmd_ = false;
        }
    }
    else
    {
        ROS_FATAL_THROTTLE(1, "Chosen controlMode is invalid");
    }

    if (publish_tf_)
    {
        const tf2::Quaternion qt(current_pose.Rot().X(), current_pose.Rot().Y(), current_pose.Rot().Z(),
                                 current_pose.Rot().W());
        const tf2::Vector3 vt(current_pose.Pos().X(), current_pose.Pos().Y(), current_pose.Pos().Z());
        const tf2::Transform base_footprint_to_odom(qt, vt);

        geometry_msgs::TransformStamped tr;
        tr.header.stamp = current_time;
        tr.header.frame_id = odometry_frame_;
        tr.child_frame_id = robot_base_frame_;
        tf2::convert(base_footprint_to_odom, tr.transform);
        transform_broadcaster_.sendTransform(tr);
    }

    if (publish_odometry_)
    {
        nav_msgs::Odometry odom;

        // Set pose covariance
        odom.pose.covariance[0] = 0.0001;
        odom.pose.covariance[7] = 0.0001;
        odom.pose.covariance[14] = 0.0001;
        odom.pose.covariance[21] = 0.0001;
        odom.pose.covariance[28] = 0.0001;
        odom.pose.covariance[35] = 0.0001;

        // Set twist covariance
        odom.twist.covariance[0] = 0.0001;
        odom.twist.covariance[7] = 0.0001;
        odom.twist.covariance[14] = 0.0001;
        odom.twist.covariance[21] = 0.0001;
        odom.twist.covariance[28] = 0.0001;
        odom.twist.covariance[35] = 0.0001;

        // publish odom topic
        odom.pose.pose.position.x = current_pose.Pos().X();
        odom.pose.pose.position.y = current_pose.Pos().Y();
        odom.pose.pose.orientation.x = current_pose.Rot().X();
        odom.pose.pose.orientation.y = current_pose.Rot().Y();
        odom.pose.pose.orientation.z = current_pose.Rot().Z();
        odom.pose.pose.orientation.w = current_pose.Rot().W();

        if (control_mode_ == "position")
        {
            odom.twist.twist.linear.x = x_;
            odom.twist.twist.linear.y = y_;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = rot_;
        }
        else
        {
            ignition::math::Vector3d linear_velocity = parent_->RelativeLinearVel();
            odom.twist.twist.linear.x = linear_velocity.X();
            odom.twist.twist.linear.y = linear_velocity.Y();
            odom.twist.twist.linear.z = linear_velocity.Z();
            ignition::math::Vector3d rot_velocity = parent_->RelativeAngularVel();
            odom.twist.twist.angular.x = rot_velocity.X();
            odom.twist.twist.angular.y = rot_velocity.Y();
            odom.twist.twist.angular.z = rot_velocity.Z();
        }

        odom.header.stamp = current_time;
        odom.header.frame_id = odometry_frame_;
        odom.child_frame_id = robot_base_frame_;

        odometry_pub_.publish(odom);
    }

    if (publish_imu_)
    {
        sensor_msgs::Imu imu;

        imu.header.stamp = current_time;
        imu.header.frame_id = robot_base_frame_;

        imu.orientation.x = current_pose.Rot().X();
        imu.orientation.y = current_pose.Rot().Y();
        imu.orientation.z = current_pose.Rot().Z();
        imu.orientation.w = current_pose.Rot().W();

        imu.orientation_covariance[0] = 0.0001;
        imu.orientation_covariance[4] = 0.0001;
        imu.orientation_covariance[8] = 0.0001;

        imu.angular_velocity.x = 0;
        imu.angular_velocity.y = 0;
        imu.angular_velocity.z = rot_;

        imu.angular_velocity_covariance[0] = 0.0001;
        imu.angular_velocity_covariance[4] = 0.0001;
        imu.angular_velocity_covariance[8] = 0.0001;

        imu.linear_acceleration.x = 0.00;
        imu.linear_acceleration.y = 0.00;
        imu.linear_acceleration.z = 9.81;

        imu.linear_acceleration_covariance[0] = 0.0001;
        imu.linear_acceleration_covariance[4] = 0.0001;
        imu.linear_acceleration_covariance[8] = 0.0001;

        imu_pub_.publish(imu);
    }

    parent_->GetWorld()->SetPaused(is_paused);
}

void PlanarMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    // Note there is no mechanism to zero cmd_vel's. move_base or cmd_vel mux should send 0
    new_cmd_ = true;
    std::lock_guard<std::mutex> lock(lock_);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
}

GZ_REGISTER_MODEL_PLUGIN(PlanarMove)
}  // namespace gazebo
