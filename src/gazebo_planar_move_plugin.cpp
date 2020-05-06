#include <boost/bind.hpp>
#include <gazebo_planar_move_plugin/gazebo_planar_move_plugin.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <cmath>
#include <string>

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

PlanarMove::PlanarMove()
    : publish_odometry_(false), publish_tf_(false), ground_truth_(false), publish_imu_(false),
      new_cmd_(false), cmd_{0, 0, 0}, tracked_state_{0, 0, 0}, gz_time_last_(0)

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
    loadParam(sdf, ground_truth_, true, std::string("ground_truth"), robot_namespace_);
    loadParam(sdf, publish_imu_, false, std::string("publish_imu"), robot_namespace_);
    loadParam(sdf, control_mode_, std::string("position"), std::string("control_mode"), robot_namespace_);

    cmd_ = {0, 0, 0};
    tracked_state_ = {0, 0, 0};

    // Get the noise params from the urdf
    if (sdf->HasElement("noise"))
    {
        if (ground_truth_)
            ROS_WARN("Ignoring odom noise as ground_truth=true");
        else
        {
            auto noise_sdf = sdf->GetElement("noise");
            if (noise_sdf->HasAttribute("type"))
            {
                const std::string type_string = noise_sdf->GetAttribute("type")->GetAsString();
                if (type_string == "gaussian")
                {
                    const double mean_x = noise_sdf->Get<double>("mean_x", 0.0).first;
                    const double mean_y = noise_sdf->Get<double>("mean_y", 0.0).first;
                    const double mean_w = noise_sdf->Get<double>("mean_w", 0.0).first;

                    const double stddev_x = noise_sdf->Get<double>("stddev_x", 0.05).first;
                    const double stddev_y = noise_sdf->Get<double>("stddev_y", 0.05).first;
                    const double stddev_w = noise_sdf->Get<double>("stddev_w", 0.05).first;

                    ROS_INFO_STREAM("Loading OdomNoise: mean=[" << mean_x << ", " << mean_y << ", " << mean_w
                                                                << "] std=[" << stddev_x << ", " << stddev_y << ", "
                                                                << stddev_w << "]");

                    dist_.reset(new OdomNoise{std::normal_distribution<double>(mean_x, stddev_x),
                                              std::normal_distribution<double>(mean_y, stddev_y),
                                              std::normal_distribution<double>(mean_w, stddev_w)});
                }
                else
                    ROS_WARN_STREAM("Noise model defined with unknown type: " << type_string
                                                                              << ". Ignoring noise model!");
            }
            else
                ROS_WARN_STREAM("No type found in noise model. Ignoring noise model!");
        }
    }

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

    tf2::Quaternion tracked_qt;
    tracked_qt.setRPY(0, 0, tracked_state_.w);

    if (publish_tf_)
    {
        geometry_msgs::TransformStamped tr;
        tr.header.stamp = current_time;
        tr.header.frame_id = odometry_frame_;
        tr.child_frame_id = robot_base_frame_;
        tr.transform.translation.x = tracked_state_.x;
        tr.transform.translation.y = tracked_state_.y;
        tr.transform.translation.z = 0.0;
        tr.transform.rotation.x = tracked_qt.x();
        tr.transform.rotation.y = tracked_qt.y();
        tr.transform.rotation.z = tracked_qt.z();
        tr.transform.rotation.w = tracked_qt.w();
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
        odom.pose.pose.position.x = tracked_state_.x;
        odom.pose.pose.position.y = tracked_state_.y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = tracked_qt.x();
        odom.pose.pose.orientation.y = tracked_qt.y();
        odom.pose.pose.orientation.z = tracked_qt.z();
        odom.pose.pose.orientation.w = tracked_qt.w();

        if (control_mode_ == "position")
        {
            odom.twist.twist.linear.x = cmd_.x;
            odom.twist.twist.linear.y = cmd_.y;
            odom.twist.twist.linear.z = 0;
            odom.twist.twist.angular.x = 0;
            odom.twist.twist.angular.y = 0;
            odom.twist.twist.angular.z = cmd_.w;
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

        imu.orientation.x = tracked_qt.x();
        imu.orientation.y = tracked_qt.y();
        imu.orientation.z = tracked_qt.z();
        imu.orientation.w = tracked_qt.w();

        imu.orientation_covariance[0] = 0.0001;
        imu.orientation_covariance[4] = 0.0001;
        imu.orientation_covariance[8] = 0.0001;

        imu.angular_velocity.x = 0;
        imu.angular_velocity.y = 0;
        imu.angular_velocity.z = cmd_.w;

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

    //
    // Position control mode
    //
    if (control_mode_ == "position")
    {
        const ignition::math::Pose3d current_pose = parent_->WorldPose();
        double current_yaw = current_pose.Rot().Yaw();

        // determine shift in position with perfect velocity tracking
        const double dx = dt * cmd_.x * cos(current_yaw) - dt * cmd_.y * cos(M_PI / 2 - current_yaw);
        const double dy = dt * cmd_.x * sin(current_yaw) + dt * cmd_.y * sin(M_PI / 2 - current_yaw);
        const double dw = dt * cmd_.w;

        const double new_x = current_pose.Pos().X() + dx;
        const double new_y = current_pose.Pos().Y() + dy;
        const double new_w = current_yaw + dw;

        // perfectly update the world position
        ignition::math::Pose3d new_pose;
        new_pose.Pos().X() = new_x;
        new_pose.Pos().Y() = new_y;
        new_pose.Rot().Euler(0.0, 0.0, new_w);

        // update tracked position
        if (ground_truth_)
        {
            tracked_state_.x = new_x;
            tracked_state_.y = new_y;
            tracked_state_.w = new_w;
        }
        else
        {
            tracked_state_.x += dx;
            tracked_state_.y += dy;
            tracked_state_.w += dw;

            // add odom sensor noise
            if (dist_)
            {
                tracked_state_.x += dx * dist_->x(generator_);
                tracked_state_.y += dy * dist_->y(generator_);
                tracked_state_.w += dw * dist_->w(generator_);
            }
        }

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
            const double yaw = pose.Rot().Yaw();
            parent_->SetLinearVel(ignition::math::Vector3d(cmd_.x * cos(yaw) - cmd_.y * sin(yaw),
                                                           cmd_.y * cos(yaw) + cmd_.x * sin(yaw), 0));
            parent_->SetAngularVel(ignition::math::Vector3d(0, 0, cmd_.w));
            new_cmd_ = false;
        }
    }
    else
    {
        ROS_FATAL_THROTTLE(1, "Chosen controlMode is invalid");
    }

    parent_->GetWorld()->SetPaused(is_paused);
}

void PlanarMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    // Note there is no mechanism to zero cmd_vel's. move_base or cmd_vel mux should send 0
    new_cmd_ = true;
    std::lock_guard<std::mutex> lock(lock_);
    cmd_ = {cmd_msg->linear.x, cmd_msg->linear.y, cmd_msg->angular.z};
}

GZ_REGISTER_MODEL_PLUGIN(PlanarMove)
}  // namespace gazebo
