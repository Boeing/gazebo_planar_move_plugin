// Copyright 2018 Boeing
#include <planar_move/planar_move.h>

#include <string>

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

    new_cmd_ = false;
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

    if (new_cmd_)
    {
        math::Pose pose = parent_->GetWorldPose();
        float yaw = pose.rot.GetYaw();
        parent_->SetLinearVel(math::Vector3(x_ * cosf(yaw) - y_ * sinf(yaw), y_ * cosf(yaw) + x_ * sinf(yaw), 0));
        parent_->SetAngularVel(math::Vector3(0, 0, rot_));
        new_cmd_ = false;
    }

    publishOdometry();
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
    new_cmd_ = true;
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
}

void PlanarMove::queueThread()
{
    const double timeout = 0.01;
    while (alive_ && rosnode_->ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void PlanarMove::publishOdometry()
{
    ros::Time current_time = ros::Time::now();

    // getting data for base_footprint to odom transform
    math::Pose pose = parent_->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_.sendTransform(tf::StampedTransform(base_footprint_to_odom, current_time, odometry_frame_, robot_base_frame_));

    // publish odom topic
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = pose.pos.x;
    odom.pose.pose.position.y = pose.pos.y;

    odom.pose.pose.orientation.x = pose.rot.x;
    odom.pose.pose.orientation.y = pose.rot.y;
    odom.pose.pose.orientation.z = pose.rot.z;
    odom.pose.pose.orientation.w = pose.rot.w;
    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;

    math::Vector3 linear_velocity = parent_->GetRelativeLinearVel();
    odom.twist.twist.linear.x = linear_velocity.x;
    odom.twist.twist.linear.y = linear_velocity.y;
    odom.twist.twist.linear.z = linear_velocity.z;

    math::Vector3 rot_velocity = parent_->GetRelativeAngularVel();
    odom.twist.twist.angular.x = rot_velocity.x;
    odom.twist.twist.angular.y = rot_velocity.y;
    odom.twist.twist.angular.z = rot_velocity.z;

    odom.header.stamp = current_time;
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = robot_base_frame_;

    odometry_pub_.publish(odom);
}

GZ_REGISTER_MODEL_PLUGIN(PlanarMove)

}  // namespace gazebo
