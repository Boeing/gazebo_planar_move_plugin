#include <gazebo_planar_move_plugin/gazebo_planar_move_plugin.h>

#include <boost/bind.hpp>
#include <cmath>
#include <geometry_msgs/Twist.h>
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
}

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
    loadParam(sdf, control_mode_, std::string("position"), std::string("control_mode"), robot_namespace_);

    x_ = 0;
    y_ = 0;
    rot_ = 0;

    nh_ = ros::NodeHandle(robot_namespace_);

    vel_sub_ = nh_.subscribe(command_topic_, 1, &PlanarMove::cmdVelCallback, this);
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    // listen to the update event (broadcast every simulation iteration)
    gz_time_last_ = 0.0;
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PlanarMove::UpdateChild, this));

    links_list_ = parent_->GetLinks();
    base_link_ = parent_->GetLink(robot_base_frame_);
}

void PlanarMove::UpdateChild()
{
    std::lock_guard<std::mutex> lock(lock_);
    const bool is_paused = parent_->GetWorld()->IsPaused();
#if GAZEBO_MAJOR_VERSION >= 8
    const bool is_physics_enabled = parent_->GetWorld()->PhysicsEnabled();
#else
    const bool is_physics_enabled = parent_->GetWorld()->GetEnablePhysicsEngine();
#endif
    parent_->GetWorld()->SetPaused(true);

    //
    // Position control mode
    //
    if (control_mode_ == "position")
    {
    // Get the simulation time and period
#if GAZEBO_MAJOR_VERSION >= 8
        double gz_time_now = parent_->GetWorld()->SimTime().Double();
        ignition::math::Pose3d current_pose = parent_->WorldPose();
        double current_yaw = current_pose.Rot().Yaw();
        ignition::math::Pose3d new_pose;
#else
        math::Pose current_pose = parent_->GetWorldPose();
        double gz_time_now = parent_->GetWorld()->GetSimTime().Double();
        double current_yaw = current_pose.rot.GetYaw();
        math::Pose new_pose;
#endif
        double dt = gz_time_now - gz_time_last_;
        gz_time_last_ = gz_time_now;

// We need to do this so dx and dy are in world frame since cmd_vel is in robot frame
#if GAZEBO_MAJOR_VERSION >= 8
        new_pose.Pos().X() = current_pose.Pos().X() + dt * x_ * cos(current_yaw);
        new_pose.Pos().Y() = current_pose.Pos().Y() + dt * x_ * sin(current_yaw);
        new_pose.Pos().X() = new_pose.Pos().X() - dt * y_ * cos(M_PI / 2 - current_yaw);
        new_pose.Pos().Y() = new_pose.Pos().Y() + dt * y_ * sin(M_PI / 2 - current_yaw);
        new_pose.Rot().Euler(0.0, 0.0, current_yaw + dt * rot_);
#else
        new_pose.pos.x = current_pose.pos.x + dt * x_ * cos(current_yaw);
        new_pose.pos.y = current_pose.pos.y + dt * x_ * sin(current_yaw);
        new_pose.pos.x = new_pose.pos.x - dt * y_ * cos(M_PI / 2 - current_yaw);
        new_pose.pos.y = new_pose.pos.y + dt * y_ * sin(M_PI / 2 - current_yaw);
        new_pose.rot.SetFromEuler(0.0, 0.0, current_yaw + dt * rot_);
#endif
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
#if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Pose3d pose = parent_->WorldPose();
            float yaw = pose.Rot().Yaw();
            parent_->SetLinearVel(
                ignition::math::Vector3d(x_ * cosf(yaw) - y_ * sinf(yaw), y_ * cosf(yaw) + x_ * sinf(yaw), 0));
            parent_->SetAngularVel(ignition::math::Vector3d(0, 0, rot_));
#else
            math::Pose pose = parent_->GetWorldPose();
            double yaw = pose.rot.GetYaw();
            parent_->SetLinearVel(
                math::Vector3(x_ * std::cos(yaw) - y_ * std::sin(yaw), y_ * std::cos(yaw) + x_ * std::sin(yaw), 0));
            parent_->SetAngularVel(math::Vector3(0, 0, rot_));
#endif
            new_cmd_ = false;
        }
    }
    else
    {
        ROS_FATAL_THROTTLE(1, "Chosen controlMode is invalid");
    }

    ros::Time current_time = ros::Time::now();

#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();
#else
    math::Pose pose = parent_->GetWorldPose();
#endif

    if (publish_tf_)
    {
#if GAZEBO_MAJOR_VERSION >= 8
        const tf2::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
        const tf2::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());
#else
        const tf2::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
        const tf2::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);
#endif
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

#if GAZEBO_MAJOR_VERSION >= 8
        // publish odom topic
        odom.pose.pose.position.x = pose.Pos().X();
        odom.pose.pose.position.y = pose.Pos().Y();
        odom.pose.pose.orientation.x = pose.Rot().X();
        odom.pose.pose.orientation.y = pose.Rot().Y();
        odom.pose.pose.orientation.z = pose.Rot().Z();
        odom.pose.pose.orientation.w = pose.Rot().W();
#else
        odom.pose.pose.position.x = pose.pos.x;
        odom.pose.pose.position.y = pose.pos.y;
        odom.pose.pose.orientation.x = pose.rot.x;
        odom.pose.pose.orientation.y = pose.rot.y;
        odom.pose.pose.orientation.z = pose.rot.z;
        odom.pose.pose.orientation.w = pose.rot.w;
#endif

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
#if GAZEBO_MAJOR_VERSION >= 8
            ignition::math::Vector3d linear_velocity = parent_->RelativeLinearVel();
            odom.twist.twist.linear.x = linear_velocity.X();
            odom.twist.twist.linear.y = linear_velocity.Y();
            odom.twist.twist.linear.z = linear_velocity.Z();
            ignition::math::Vector3d rot_velocity = parent_->RelativeAngularVel();
            odom.twist.twist.angular.x = rot_velocity.X();
            odom.twist.twist.angular.y = rot_velocity.Y();
            odom.twist.twist.angular.z = rot_velocity.Z();
#else
            math::Vector3 linear_velocity = parent_->GetWorldLinearVel();
            odom.twist.twist.linear.x = linear_velocity.x;
            odom.twist.twist.linear.y = linear_velocity.y;
            odom.twist.twist.linear.z = linear_velocity.z;
            math::Vector3 rot_velocity = parent_->GetWorldAngularVel();
            odom.twist.twist.angular.x = rot_velocity.x;
            odom.twist.twist.angular.y = rot_velocity.y;
            odom.twist.twist.angular.z = rot_velocity.z;
#endif
        }

        odom.header.stamp = current_time;
        odom.header.frame_id = odometry_frame_;
        odom.child_frame_id = robot_base_frame_;

        odometry_pub_.publish(odom);
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
}