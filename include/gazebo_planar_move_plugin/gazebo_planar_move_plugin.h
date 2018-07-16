// Copyright 2018 Boeing
#ifndef GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H
#define GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H

#include <atomic>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <mutex>
#include <sdf/sdf.hh>
#include <string>
#include <thread>
#include <vector>

#include <nav_msgs/Odometry.h>

#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <tf/transform_broadcaster.h>

namespace gazebo
{

class PlanarMove : public ModelPlugin
{
  public:
    PlanarMove();
    ~PlanarMove();
    void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);

  protected:
    virtual void UpdateChild();

  private:
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& cmd_msg);

    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;

    ros::NodeHandle nh_;
    ros::Publisher odometry_pub_;
    ros::Subscriber vel_sub_;

    tf::TransformBroadcaster transform_broadcaster_;

    std::mutex lock_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;
    std::string control_mode_;

    bool publish_odometry_;
    bool publish_tf_;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    std::thread callback_queue_thread_;
    void queueThread();
    std::atomic<bool> new_cmd_;
    double x_;
    double y_;
    double rot_;

    bool alive_;

    double gz_time_last_;

    std::vector<physics::LinkPtr> links_list_;
    physics::LinkPtr base_link_;
};

}  // namespace gazebo

#endif  // GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H
