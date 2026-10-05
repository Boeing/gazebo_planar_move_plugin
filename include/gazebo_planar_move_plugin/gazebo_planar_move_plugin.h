#ifndef GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H
#define GAZEBO_PLANAR_MOVE_PLANAR_MOVE_H

#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <atomic>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <mutex>
#include <random>
#include <sdf/Element.hh>
#include <string>
#include <thread>
#include <vector>

namespace gz::sim::systems
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

class PlanarMove : public gz::sim::System,
                   public gz::sim::ISystemConfigure,
                   public gz::sim::ISystemPreUpdate
{
  public:
    PlanarMove() = default;
    ~PlanarMove() override;

    void Configure(const gz::sim::Entity& _entity,
                   const std::shared_ptr<const sdf::Element>& _sdf,
                   gz::sim::EntityComponentManager& _ecm,
                   gz::sim::EventManager& _eventMgr) override;

    void PreUpdate(const gz::sim::UpdateInfo& _info,
                   gz::sim::EntityComponentManager& _ecm) override;

  private:
    void cmdVelCallback(const geometry_msgs::msg::Twist& cmd_msg);

    gz::sim::Entity model_entity_{gz::sim::kNullEntity};
    gz::sim::Model model_;
    gz::sim::Link base_link_;

    rclcpp::Node::SharedPtr ros_node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::thread spin_thread_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr vel_sub_;

    std::shared_ptr<tf2_ros::TransformBroadcaster> transform_broadcaster_;

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

    mutable std::mutex cmd_lock;
    CmdVel cmd_;

    State2D tracked_state_;
    double publish_rate_;
    double publish_period_;
    double update_rate_;
    double update_period_;

    double last_update_time_;
    double last_publish_time_;

    double drift_x = 0.0;
    double drift_y = 0.0;
    double drift_w = 0.0;
};
}  // namespace gz::sim::systems

#endif
