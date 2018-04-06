// Copyright 2018 Boeing
#include <gazebo_planar_move_plugin/gazebo_planar_move_plugin.h>

#include <boost/bind.hpp>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <string>

#include <ros/advertise_options.h>

namespace gazebo
{

PlanarMove::PlanarMove() : x_(0), y_(0), rot_(0), alive_(false), gz_time_last_(0)

{
}

PlanarMove::~PlanarMove()
{
    alive_ = false;
    queue_.clear();
    queue_.disable();
    callback_queue_thread_.join();
}

// cppcheck-suppress unusedFunction
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
        ROS_WARN_NAMED("planar_move",
                       "PlanarMovePlugin (ns = %s) missing <odometryTopic>, "
                       "defaults to \"%s\"",
                       robot_namespace_.c_str(), odometry_topic_.c_str());
    }
    else
    {
        odometry_topic_ = sdf->GetElement("odometryTopic")->Get<std::string>();
    }

    odometry_frame_ = "odom";
    if (!sdf->HasElement("odometryFrame"))
    {
        ROS_WARN_NAMED("planar_move",
                       "PlanarMovePlugin (ns = %s) missing <odometryFrame>, "
                       "defaults to \"%s\"",
                       robot_namespace_.c_str(), odometry_frame_.c_str());
    }
    else
    {
        odometry_frame_ = sdf->GetElement("odometryFrame")->Get<std::string>();
    }

    robot_base_frame_ = "base_link";
    if (!sdf->HasElement("robotBaseFrame"))
    {
        ROS_WARN_NAMED("planar_move",
                       "PlanarMovePlugin (ns = %s) missing <robotBaseFrame>, "
                       "defaults to \"%s\"",
                       robot_namespace_.c_str(), robot_base_frame_.c_str());
    }
    else
    {
        robot_base_frame_ = sdf->GetElement("robotBaseFrame")->Get<std::string>();
    }

    control_mode_ = "position";
    if (!sdf->HasElement("controlMode"))
    {
        ROS_WARN_NAMED("planar_move", "PlanarMovePlugin (ns = %s) missing <controlMode>, defaults to \"%s\"",
                       control_mode_.c_str(), control_mode_.c_str());
    }
    else
    {
        control_mode_ = sdf->GetElement("controlMode")->Get<std::string>();
    }

    x_ = 0;
    y_ = 0;
    rot_ = 0;
    alive_ = true;

    // Ensure that ROS has been initialized and subscribe to cmd_vel
    if (!ros::isInitialized())
    {
        ROS_FATAL_STREAM_NAMED("planar_move",
                               "PlanarMovePlugin (ns = " << robot_namespace_
                                                         << "). A ROS node for Gazebo has not been initialized, "
                                                         << "unable to load plugin. Load the Gazebo system plugin "
                                                         << "'libgazebo_ros_api_plugin.so' in the gazebo_ros package)");
        return;
    }
    nh_ = ros::NodeHandle(robot_namespace_);

    // subscribe to the cmd_vel topic
    ros::SubscribeOptions so = ros::SubscribeOptions::create<geometry_msgs::Twist>(
        command_topic_, 1, boost::bind(&PlanarMove::cmdVelCallback, this, _1), ros::VoidPtr(), &queue_);

    vel_sub_ = nh_.subscribe(so);
    odometry_pub_ = nh_.advertise<nav_msgs::Odometry>(odometry_topic_, 1);

    callback_queue_thread_ = std::thread(&PlanarMove::queueThread, this);

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
    parent_->GetWorld()->SetPaused(true);

    if (control_mode_ == "position")  // Position control mode
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

// ROS_INFO_STREAM("Command: x:" << x_ << " y:" << y_ << " rotZ:" << rot_);
// ROS_INFO_STREAM("Current Pose: x:" <<  current_pose.pos.x << " y:" <<
// current_pose.pos.y << " Yaw:" << current_pose.rot.GetYaw());

// Forward velocity component (x)
#if GAZEBO_MAJOR_VERSION >= 8
        new_pose.Pos().X() = current_pose.Pos().X() + dt * x_ * cos(current_yaw);  // We need to do this so dx
                                                                                   // and dy are in world
                                                                                   // frame
        new_pose.Pos().Y() = current_pose.Pos().Y() + dt * x_ * sin(current_yaw);  // since cmd_vel is in robot frame

        // Strafing velocity component (y)
        new_pose.Pos().X() =
            new_pose.Pos().X() - dt * y_ * cos(M_PI / 2 - current_yaw);  // We need to do this so dx and
                                                                         // dy are in world frame
        new_pose.Pos().Y() =
            new_pose.Pos().Y() + dt * y_ * sin(M_PI / 2 - current_yaw);  // since cmd_vel is in robot frame
        new_pose.Rot().Euler(0.0, 0.0, current_yaw + dt * rot_);
#else
        new_pose.pos.x = current_pose.pos.x + dt * x_ * cos(current_yaw);  // We need to do this so dx and
                                                                           // dy are in world frame
        new_pose.pos.y = current_pose.pos.y + dt * x_ * sin(current_yaw);  // since cmd_vel is in robot frame

        // Strafing velocity component (y)
        new_pose.pos.x = new_pose.pos.x - dt * y_ * cos(M_PI / 2 - current_yaw);  // We need to do
                                                                                  // this so dx
                                                                                  // and dy are in
                                                                                  // world frame
        new_pose.pos.y = new_pose.pos.y + dt * y_ * sin(M_PI / 2 - current_yaw);  // since cmd_vel is in robot frame
        new_pose.rot.SetFromEuler(0.0, 0.0, current_yaw + dt * rot_);
#endif
        if (base_link_ == NULL)
        {
            ROS_FATAL_THROTTLE(1, "Model has no link named 'base link'");
        }
        else
        {
            // ROS_INFO_STREAM("Setting link, dt is:" << dt << " new x:" <<
            // new_pose.pos.x << " new y:" << new_pose.pos.y << " new yaw:" <<
            // new_pose.rot.GetYaw());
            parent_->SetLinkWorldPose(new_pose, base_link_);

            // Hack disables physics, required after call to any physics related
            // function call
            for (physics::LinkPtr link : links_list_)
            {
                link->SetEnabled(false);
            }
        }
    }
    else if (control_mode_ == "velocity")  // Velocity control mode
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
            parent_->SetLinearVel(math::Vector3(x_ * cosf(yaw) - y_ * sinf(yaw), y_ * cosf(yaw) + x_ * sinf(yaw), 0));
            parent_->SetAngularVel(math::Vector3(0, 0, rot_));
#endif
            new_cmd_ = false;
        }
    }
    else
    {
        ROS_FATAL_THROTTLE(1, "Chosen controlMode is invalid");
    }

    parent_->GetWorld()->SetPaused(is_paused);

    publishOdometry();
}

void PlanarMove::cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg)
{
    // Note there is no mechanism to zero cmd_vel's. move_base or cmd_vel mux
    // should send 0
    new_cmd_ = true;
    std::lock_guard<std::mutex> lock(lock_);
    x_ = cmd_msg->linear.x;
    y_ = cmd_msg->linear.y;
    rot_ = cmd_msg->angular.z;
}

void PlanarMove::queueThread()
{
    const double timeout = 0.01;
    while (alive_ && nh_.ok())
    {
        queue_.callAvailable(ros::WallDuration(timeout));
    }
}

void PlanarMove::publishOdometry()
{
    ros::Time current_time = ros::Time::now();

// getting data for base_footprint to odom transform
#if GAZEBO_MAJOR_VERSION >= 8
    ignition::math::Pose3d pose = parent_->WorldPose();

    tf::Quaternion qt(pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W());
    tf::Vector3 vt(pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z());

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_.sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odometry_frame_, robot_base_frame_));

    // publish odom topic
    nav_msgs::Odometry odom;
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();

    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.pose.covariance[0] = 0.00001;
    odom.pose.covariance[7] = 0.00001;
    odom.pose.covariance[14] = 1000000000000.0;
    odom.pose.covariance[21] = 1000000000000.0;
    odom.pose.covariance[28] = 1000000000000.0;
    odom.pose.covariance[35] = 0.001;

    ignition::math::Vector3d linear_velocity = parent_->RelativeLinearVel();
    odom.twist.twist.linear.x = linear_velocity.X();
    odom.twist.twist.linear.y = linear_velocity.Y();
    odom.twist.twist.linear.z = linear_velocity.Z();

    ignition::math::Vector3d rot_velocity = parent_->RelativeAngularVel();
    odom.twist.twist.angular.x = rot_velocity.X();
    odom.twist.twist.angular.y = rot_velocity.Y();
    odom.twist.twist.angular.z = rot_velocity.Z();

#else
    math::Pose pose = parent_->GetWorldPose();

    tf::Quaternion qt(pose.rot.x, pose.rot.y, pose.rot.z, pose.rot.w);
    tf::Vector3 vt(pose.pos.x, pose.pos.y, pose.pos.z);

    tf::Transform base_footprint_to_odom(qt, vt);
    transform_broadcaster_.sendTransform(
        tf::StampedTransform(base_footprint_to_odom, current_time, odometry_frame_, robot_base_frame_));

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
#endif

    odom.header.stamp = current_time;
    odom.header.frame_id = odometry_frame_;
    odom.child_frame_id = robot_base_frame_;

    odometry_pub_.publish(odom);
}

GZ_REGISTER_MODEL_PLUGIN(PlanarMove)

}  // namespace gazebo
