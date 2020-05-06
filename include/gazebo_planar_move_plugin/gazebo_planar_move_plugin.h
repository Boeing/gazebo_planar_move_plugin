#ifndef GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H
#define GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H

#include <nav_msgs/Odometry.h>
#include <ros/callback_queue.h>
#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <mutex>
#include <random>
#include <sdf/sdf.hh>
#include <string>
#include <thread>
#include <vector>

namespace gazebo
{

struct State2D
{
    double x;
    double y;
    double w;
};

struct CmdVel
{
    double x;
    double y;
    double w;
};

struct OdomNoise
{
    std::normal_distribution<double> x;
    std::normal_distribution<double> y;
    std::normal_distribution<double> w;
};

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
    ros::Publisher imu_pub_;
    ros::Subscriber vel_sub_;

    tf2_ros::TransformBroadcaster transform_broadcaster_;

    std::mutex lock_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;
    std::string control_mode_;

    std::default_random_engine generator_;
    std::unique_ptr<OdomNoise> dist_;

    bool publish_odometry_;
    bool publish_tf_;
    bool ground_truth_;
    bool publish_imu_;

    std::atomic<bool> new_cmd_;
    CmdVel cmd_;

    State2D tracked_state_;

    double gz_time_last_;

    std::vector<physics::LinkPtr> links_list_;
    physics::LinkPtr base_link_;
};
}  // namespace gazebo

#endif
