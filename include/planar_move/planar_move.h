#ifndef PLANAR_MOVE_H
#define PLANAR_MOVE_H

#include <mutex>
#include <thread>

#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <nav_msgs/Odometry.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

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
    virtual void FiniChild();

  private:
    void publishOdometry(double step_time);
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr &cmd_msg);

    physics::ModelPtr parent_;
    event::ConnectionPtr update_connection_;

    std::unique_ptr<ros::NodeHandle> rosnode_;
    ros::Publisher odometry_pub_;
    ros::Subscriber vel_sub_;

    tf::TransformBroadcaster transform_broadcaster_;

    nav_msgs::Odometry odom_;

    std::mutex lock_;

    std::string robot_namespace_;
    std::string command_topic_;
    std::string odometry_topic_;
    std::string odometry_frame_;
    std::string robot_base_frame_;
    double odometry_rate_;

    // Custom Callback Queue
    ros::CallbackQueue queue_;
    std::thread callback_queue_thread_;
    void queueThread();

    double x_;
    double y_;
    double rot_;
    bool alive_;
    common::Time last_odom_publish_time_;
    math::Pose last_odom_pose_;
};

}  // namespace gazebo

#endif  // PLANAR_MOVE_H
