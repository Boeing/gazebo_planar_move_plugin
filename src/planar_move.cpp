#include <planar_move/planar_move.h>

#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>

#include <ros/advertise_options.h>

namespace gazebo
{

PlanarMove::PlanarMove()
{
}

PlanarMove::~PlanarMove()
{
}

void PlanarMove::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    ROS_INFO("Initialising PlanarMove Plugin");

    parent_ = parent;

    robot_namespace_ = "";
    if (!sdf->HasElement("robotNamespace"))
    {
        ROS_INFO_NAMED("planar_move", "PlanarMovePlugin missing <robotNamespace>, defaults to \"%s\"",
                       robot_namespace_.c_str());
    }
    else
    {
        robot_namespace_ = sdf->GetElement("robotNamespace")->Get<std::string>();
    }

    command_topic_ = "cmd_vel";
    if (!sdf->HasElement("commandTopic"))
    {
        ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <commandTopic>, defaults to \"%s\"",
                       robot_namespace_.c_str(), command_topic_.c_str());
    }
    else
    {
        command_topic_ = sdf->GetElement("commandTopic")->Get<std::string>();
    }

    odometry_topic_ = "odom";
    if (!sdf->HasElement("odometryTopic"))
    {
        ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryTopic>, defaults to \"%s\"",
                       robot_namespace_.c_str(), odometry_topic_.c_str());
    }
    else
    {
        odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
        ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryFrame>, defaults to \"%s\"",
                       robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else
    {
        odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_link";
    if (!sdf->HasElement("robotBaseFrame"))
    {
        ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, defaults to \"%s\"",
                       robot_namespace_.c_str(), robot_base_frame_.c_str());
    }
    else
    {
        robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    odometry_rate_ = 20.0;
    if (!sdf->HasElement("odometryRate"))
    {
        ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <odometryRate>, defaults to %f",
                       robot_namespace_.c_str(), odometry_rate_);
    }
    else
    {
        odometry_rate_ = sdf->GetElement("odometryRate")->Get<double>();
    }

    last_odom_publish_time_ = parent_->GetWorld()->GetSimTime();
    last_odom_pose_ = parent_->GetWorldPose();
    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("planar_move", "PlanarMovePlugin (ns = " << robot_namespace_
                                                                        << "). A ROS node for Gazebo has not been initialized, "
                                                                        << "unable to load plugin. Load the Gazebo system plugin "
                                                                        << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    rosnode_.reset(new ros::NodeHandle(robot_namespace_));

    // subscribe to the odometry topic
    ros::SubscribeOptions so =
        ros::SubscribeOptions::create<geometry_msgs::Twist>(command_topic_, 1,
                                                            boost::bind(&PlanarMove::cmdVelCallback, this, _1),
                                                            ros::VoidPtr(), &queue_);

    vel_sub_ = rosnode_->subscribe(so);
    odometry_pub_ = rosnode_->advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    callback_queue_thread_ = std::thread(&PlanarMove::queueThread, this);

    // listen to the update event (broadcast every simulation iteration)
    update_connection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&PlanarMove::UpdateChild, this));
}

void PlanarMove::UpdateChild()
{
    std::lock_guard<std::mutex> lock(lock_);

    math::Pose pose = parent_->GetWorldPose();
    float yaw = pose.rot.GetYaw();
    parent_->SetLinearVel(math::Vector3(x_ * cosf(yaw) - y_ * sinf(yaw), y_ * cosf(yaw) + x_ * sinf(yaw), 0));
    parent_->SetAngularVel(math::Vector3(0, 0, rot_));

    if (odometry_rate_ > 0.0)
    {
        common::Time current_time = parent_->GetWorld()->GetSimTime();
        double seconds_since_last_update = (current_time - last_odom_publish_time_).Double();
        if (seconds_since_last_update > (1.0 / odometry_rate_))
        {
            publishOdometry(seconds_since_last_update);
            last_odom_publish_time_ = current_time;
        }
    }
}

void PlanarMove::FiniChild()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    rosnode_->shutdown();
    callback_queue_thread_.join();
}

void PlanarMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg)
{
    std::lock_guard<std::mutex> lock(lock_);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
}

void PlanarMove::queueThread()
{
    static const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void PlanarMove::publishOdometry(double step_time)
{
    ros::Time current_time = ros::Time::now();

    // getting data for base_footprint to odom transform
    math::Pose pose = parent_->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_.sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odometry_frame_, robot_base_frame_));

    // publish odom topic
    odom_.pose.pose.position.x = pose.pos.x;
    odom_.pose.pose.position.y = pose.pos.y;

    odom_.pose.pose.orientation.x = pose.rot.x;
    odom_.pose.pose.orientation.y = pose.rot.y;
    odom_.pose.pose.orientation.z = pose.rot.z;
    odom_.pose.pose.orientation.w = pose.rot.w;
    odom_.pose.covariance[0] = 0.00001;
    odom_.pose.covariance[7] = 0.00001;
    odom_.pose.covariance[14] = 1000000000000.0;
    odom_.pose.covariance[21] = 1000000000000.0;
    odom_.pose.covariance[28] = 1000000000000.0;
    odom_.pose.covariance[35] = 0.001;

    // get velocity in /odom frame
    math::Vector3 linear;
    linear.x = (pose.pos.x - last_odom_pose_.pos.x) / step_time;
    linear.y = (pose.pos.y - last_odom_pose_.pos.y) / step_time;
    if (rot_ > M_PI / step_time)
    {
        // we cannot calculate the angular velocity correctly
        odom_.twist.twist.angular.z = rot_;
    }
    else
    {
        float last_yaw = last_odom_pose_.rot.GetYaw();
        float current_yaw = pose.rot.GetYaw();
        while (current_yaw < last_yaw - M_PI)
            current_yaw += 2 * M_PI;
        while (current_yaw > last_yaw + M_PI)
            current_yaw -= 2 * M_PI;
        float angular_diff = current_yaw - last_yaw;
        odom_.twist.twist.angular.z = angular_diff / step_time;
    }
    last_odom_pose_ = pose;

    // convert velocity to child_frame_id (aka base_footprint)
    float yaw = pose.rot.GetYaw();
    odom_.twist.twist.linear.x = cosf(yaw) * linear.x + sinf(yaw) * linear.y;
    odom_.twist.twist.linear.y = cosf(yaw) * linear.y - sinf(yaw) * linear.x;

    odom_.header.stamp = current_time;
    odom_.header.frame_id = odometry_frame_;
    odom_.child_frame_id = robot_base_frame_;

    odometry_pub_.publish(odom_);
}

GZ_REGISTER_MODEL_PLUGIN(PlanarMove)

}  // namespace gazebo
