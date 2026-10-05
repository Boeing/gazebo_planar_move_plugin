#include <boost/bind.hpp>
#include <gazebo_planar_move_plugin/gazebo_planar_move_plugin.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <chrono>
#include <cmath>
#include <string>

namespace gz::sim::systems
{
namespace
{

template <typename TYPE>
void loadParam(const std::shared_ptr<const sdf::Element>& sdf, TYPE& value, const TYPE& default_value,
               const std::string& param_name, const std::string& robot_namespace)
{
    if (!sdf || !sdf->HasElement(param_name))
    {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "PlanarMovePlugin (ns = " << robot_namespace << ") missing <"
                                                                                   << param_name << ">, defaults to \""
                                                                                   << default_value << "\"");
        value = default_value;
    }
    else
    {
        value = sdf->GetElementImpl(param_name)->Get<TYPE>();
    }
}



rclcpp::Time simTimeToRosTime(const double sim_seconds)
{
    return rclcpp::Time(static_cast<int64_t>(sim_seconds * 1e9), RCL_ROS_TIME);
}
}  // namespace

PlanarMove::~PlanarMove()
{
  // Stop the ROS executor cleanly
  if (executor_) {
    executor_->cancel();
  }
  if (spin_thread_.joinable()) {
    spin_thread_.join();
  }
}

void PlanarMove::Configure(const gz::sim::Entity& _entity,
                          const std::shared_ptr<const sdf::Element>& _sdf,
                          gz::sim::EntityComponentManager& _ecm,
                          gz::sim::EventManager& _eventMgr)
{
    (void)_ecm;
    (void)_eventMgr;

    model_entity_ = _entity;
    model_ = gz::sim::Model(model_entity_);

    loadParam(_sdf, robot_namespace_, std::string("/"), std::string("robot_namespace"), robot_namespace_);
    loadParam(_sdf, command_topic_, std::string("cmd_vel"), std::string("command_topic"), robot_namespace_);
    loadParam(_sdf, odometry_topic_, std::string("odom"), std::string("odometry_topic"), robot_namespace_);
    loadParam(_sdf, odometry_frame_, std::string("odom"), std::string("odometry_frame"), robot_namespace_);
    loadParam(_sdf, robot_base_frame_, std::string("base_link"), std::string("robot_frame"), robot_namespace_);
    loadParam(_sdf, publish_odometry_, true, std::string("publish_odometry"), robot_namespace_);
    loadParam(_sdf, publish_tf_, true, std::string("publish_tf"), robot_namespace_);
    loadParam(_sdf, ground_truth_, true, std::string("ground_truth"), robot_namespace_);
    loadParam(_sdf, publish_imu_, false, std::string("publish_imu"), robot_namespace_);
    loadParam(_sdf, control_mode_, std::string("position"), std::string("control_mode"), robot_namespace_);
    loadParam(_sdf, update_rate_, 60.0, std::string("update_rate"), robot_namespace_);
    update_period_ = 1.0 / update_rate_;
    loadParam(_sdf, publish_rate_, 30.0, std::string("publish_rate"), robot_namespace_);
    publish_period_ = 1.0 / publish_rate_;

    cmd_ = {0.0, 0.0, 0.0};
    tracked_state_ = {0.0, 0.0, 0.0};

    if (_sdf && _sdf->HasElement("noise"))
    {
        if (ground_truth_)
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Ignoring odom noise as ground_truth=true");
        else
        {
            auto noise_sdf = _sdf->GetElementImpl("noise");
            if (noise_sdf->HasAttribute("type"))
            {
                const std::string type_string = noise_sdf->GetAttribute("type")->GetAsString();
                if (type_string == "gaussian")
                {
                    drift_x = noise_sdf->Get<double>("drift_x", 0.05).first;
                    drift_y = noise_sdf->Get<double>("drift_y", 0.05).first;
                    drift_w = noise_sdf->Get<double>("drift_w", 0.05).first;

                    const double mean_x = noise_sdf->Get<double>("mean_x", 0.0).first;
                    const double mean_y = noise_sdf->Get<double>("mean_y", 0.0).first;
                    const double mean_w = noise_sdf->Get<double>("mean_w", 0.0).first;

                    const double stddev_x = noise_sdf->Get<double>("stddev_x", 0.05).first;
                    const double stddev_y = noise_sdf->Get<double>("stddev_y", 0.05).first;
                    const double stddev_w = noise_sdf->Get<double>("stddev_w", 0.05).first;

                    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"), "Loading OdomNoise: mean=["
                                                                         << mean_x << ", " << mean_y << ", " << mean_w
                                                                         << "] std=[" << stddev_x << ", " << stddev_y
                                                                         << ", " << stddev_w << "]");

                    dist_.reset(new OdomNoise{std::normal_distribution<double>(mean_x, stddev_x),
                                              std::normal_distribution<double>(mean_y, stddev_y),
                                              std::normal_distribution<double>(mean_w, stddev_w)});
                }
                else
                {
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Noise model defined with unknown type: "
                                                                         << type_string << ". Ignoring noise model!");
                }
            }
            else
            {
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "No type found in noise model. Ignoring noise model!");
            }
        }
    }

    if (model_entity_ == gz::sim::kNullEntity)
    {
        return;
    }

    if (!rclcpp::ok()) {
        // gz-sim may not have called rclcpp::init; do it ourselves.
        rclcpp::init(0, nullptr);
    }

    ros_node_ = std::make_shared<rclcpp::Node>("gz_planar_move_plugin");
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        command_topic_, qos,
        std::bind(&PlanarMove::cmdVelCallback, this, std::placeholders::_1));
    odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_, qos);
    imu_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>("imu", qos);

    const auto base_entity = model_.LinkByName(_ecm, robot_base_frame_);
    if (base_entity != gz::sim::kNullEntity)
    {
        base_link_ = gz::sim::Link(base_entity);
    }

    last_update_time_ = 0.0;
    last_publish_time_ = 0.0;

    // ── 4. Spin the executor in a background thread ──────────
    executor_ =
        std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(ros_node_);

    spin_thread_ = std::thread([this]() {
        executor_->spin();
    });
}

void PlanarMove::PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm)
{
    if (model_entity_ == gz::sim::kNullEntity || !model_.Valid(_ecm))
    {
        return;
    }

    CmdVel last_cmd;
    {
        std::unique_lock<std::mutex> lock(cmd_lock);
        last_cmd = cmd_;
    }

    const double gz_time_now = std::chrono::duration<double>(_info.simTime).count();
    tf2::Quaternion tracked_qt;
    tracked_qt.setRPY(0.0, 0.0, tracked_state_.w);

    const double dt_since_last_publish = gz_time_now - last_publish_time_;
    if (dt_since_last_publish >= publish_period_)
    {
        const rclcpp::Time current_time = simTimeToRosTime(gz_time_now);

        if (publish_tf_)
        {
            geometry_msgs::msg::TransformStamped tr;
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
            transform_broadcaster_->sendTransform(tr);
        }

        if (publish_odometry_)
        {
            nav_msgs::msg::Odometry odom;
            odom.pose.covariance[0] = 0.0001;
            odom.pose.covariance[7] = 0.0001;
            odom.pose.covariance[14] = 0.0001;
            odom.pose.covariance[21] = 0.0001;
            odom.pose.covariance[28] = 0.0001;
            odom.pose.covariance[35] = 0.0001;

            odom.twist.covariance[0] = 0.0001;
            odom.twist.covariance[7] = 0.0001;
            odom.twist.covariance[14] = 0.0001;
            odom.twist.covariance[21] = 0.0001;
            odom.twist.covariance[28] = 0.0001;
            odom.twist.covariance[35] = 0.0001;

            odom.pose.pose.position.x = tracked_state_.x;
            odom.pose.pose.position.y = tracked_state_.y;
            odom.pose.pose.position.z = 0.0;
            odom.pose.pose.orientation.x = tracked_qt.x();
            odom.pose.pose.orientation.y = tracked_qt.y();
            odom.pose.pose.orientation.z = tracked_qt.z();
            odom.pose.pose.orientation.w = tracked_qt.w();

            if (control_mode_ == "position")
            {
                odom.twist.twist.linear.x = last_cmd.x;
                odom.twist.twist.linear.y = last_cmd.y;
                odom.twist.twist.linear.z = 0.0;
                odom.twist.twist.angular.x = 0.0;
                odom.twist.twist.angular.y = 0.0;
                odom.twist.twist.angular.z = last_cmd.w;
            }
            else
            {
                const auto current_link_pose = base_link_.WorldPose(_ecm);
                if (current_link_pose)
                {
                    const auto linear_vel = base_link_.WorldLinearVelocity(_ecm);
                    const auto angular_vel = base_link_.WorldAngularVelocity(_ecm);
                    if (linear_vel)
                    {
                        odom.twist.twist.linear.x = linear_vel->X();
                        odom.twist.twist.linear.y = linear_vel->Y();
                        odom.twist.twist.linear.z = linear_vel->Z();
                    }
                    if (angular_vel)
                    {
                        odom.twist.twist.angular.x = angular_vel->X();
                        odom.twist.twist.angular.y = angular_vel->Y();
                        odom.twist.twist.angular.z = angular_vel->Z();
                    }
                }
            }

            odom.header.stamp = current_time;
            odom.header.frame_id = odometry_frame_;
            odom.child_frame_id = robot_base_frame_;
            odometry_pub_->publish(odom);
        }

        if (publish_imu_)
        {
            sensor_msgs::msg::Imu imu;
            imu.header.stamp = current_time;
            imu.header.frame_id = robot_base_frame_;
            imu.orientation.x = tracked_qt.x();
            imu.orientation.y = tracked_qt.y();
            imu.orientation.z = tracked_qt.z();
            imu.orientation.w = tracked_qt.w();
            imu.orientation_covariance[0] = 0.0001;
            imu.orientation_covariance[4] = 0.0001;
            imu.orientation_covariance[8] = 0.0001;
            imu.angular_velocity.x = 0.0;
            imu.angular_velocity.y = 0.0;
            imu.angular_velocity.z = last_cmd.w;
            imu.angular_velocity_covariance[0] = 0.0001;
            imu.angular_velocity_covariance[4] = 0.0001;
            imu.angular_velocity_covariance[8] = 0.0001;
            imu.linear_acceleration.x = 0.0;
            imu.linear_acceleration.y = 0.0;
            imu.linear_acceleration.z = 9.81;
            imu_pub_->publish(imu);
        }

        last_publish_time_ = gz_time_now;
    }

    const double dt_since_last_update = gz_time_now - last_update_time_;
    if (dt_since_last_update >= update_period_)
    {
        if (control_mode_ == "position")
        {
            const auto current_pose = base_link_.WorldPose(_ecm);
            if (!current_pose)
            {
                return;
            }

            const double current_yaw = current_pose->Rot().Yaw();
            const double dx = dt_since_last_update * last_cmd.x * std::cos(current_yaw) -
                              dt_since_last_update * last_cmd.y * std::cos(M_PI / 2.0 - current_yaw);
            const double dy = dt_since_last_update * last_cmd.x * std::sin(current_yaw) +
                              dt_since_last_update * last_cmd.y * std::sin(M_PI / 2.0 - current_yaw);
            const double dw = dt_since_last_update * last_cmd.w;

            const double new_x = current_pose->Pos().X() + dx;
            const double new_y = current_pose->Pos().Y() + dy;
            const double new_w = current_yaw + dw;

            gz::math::Pose3d new_pose(new_x, new_y, 0.0, 0.0, 0.0, new_w);

            if (ground_truth_)
            {
                tracked_state_.x = new_x;
                tracked_state_.y = new_y;
                tracked_state_.w = new_w;
            }
            else
            {
                const double x_error = dx * drift_x;
                const double y_error = dy * drift_y;
                const double w_error = dw * drift_w;

                if (dist_)
                {
                    tracked_state_.x += x_error * dist_->x(generator_);
                    tracked_state_.y += y_error * dist_->y(generator_);
                    tracked_state_.w += w_error * dist_->w(generator_);
                }
                else
                {
                    tracked_state_.x += x_error;
                    tracked_state_.y += y_error;
                    tracked_state_.w += w_error;
                }
            }

            model_.SetWorldPoseCmd(_ecm, new_pose);
        }
        else if (control_mode_ == "velocity")
        {
            const gz::math::Vector3d linear(last_cmd.x, last_cmd.y, 0.0);
            const gz::math::Vector3d angular(0.0, 0.0, last_cmd.w);
            base_link_.SetLinearVelocity(_ecm, linear);
            base_link_.SetAngularVelocity(_ecm, angular);

            std::unique_lock<std::mutex> lock(cmd_lock);
        }
        else
        {
            RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Chosen controlMode is invalid");
        }

        last_update_time_ = gz_time_now;
    }
}

void PlanarMove::cmdVelCallback(const geometry_msgs::msg::Twist& cmd_msg)
{
    RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Got new Twist message");
    std::unique_lock<std::mutex> lock(cmd_lock);
    cmd_ = {cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z};
}

}  // namespace gz::sim::systems

GZ_ADD_PLUGIN(gz::sim::systems::PlanarMove, gz::sim::System,
              gz::sim::ISystemConfigure, gz::sim::ISystemPreUpdate)
