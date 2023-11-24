#include <boost/bind.hpp>
#include <gazebo_planar_move_plugin/gazebo_planar_move_plugin.h>
#include <gazebo_ros/conversions/builtin_interfaces.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

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
        RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "PlanarMovePlugin (ns = " << robot_namespace << ") missing <"
                                                                                   << param_name << ">, defaults to \""
                                                                                   << default_value << "\"");
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
      new_cmd_(false), cmd_{0, 0, 0}, tracked_state_{0, 0, 0}, publish_rate_(30.0), update_rate_(50.0),
      last_update_time_(0.0), last_publish_time_(0.0)
{
}

PlanarMove::~PlanarMove()
{
}

// cppcheck-suppress unusedFunction
void PlanarMove::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
    model_ = parent;

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
    loadParam(sdf, update_rate_, 60.0, std::string("update_rate"), robot_namespace_);
    update_period_ = 1.0 / update_rate_;
    loadParam(sdf, publish_rate_, 30.0, std::string("publish_rate"), robot_namespace_);
    publish_period_ = 1.0 / publish_rate_;
    cmd_ = {0, 0, 0};
    tracked_state_ = {0, 0, 0};

    // Get the noise params from the urdf
    if (sdf->HasElement("noise"))
    {
        if (ground_truth_)
            RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Ignoring odom noise as ground_truth=true");
        else
        {
            auto noise_sdf = sdf->GetElement("noise");
            if (noise_sdf->HasAttribute("type"))
            {
                const std::string type_string = noise_sdf->GetAttribute("type")->GetAsString();
                if (type_string == "gaussian")
                {
                    drift_x = noise_sdf->Get<double>("drift_x", 0.05).first;
                    drift_y = noise_sdf->Get<double>("drift_y", 0.05).first;
                    drift_w = noise_sdf->Get<double>("drift_w", 0.05).first;  // 2.5 deg per rad

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
                    RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "Noise model defined with unknown type: "
                                                                         << type_string << ". Ignoring noise model!");
            }
            else
                RCLCPP_WARN_STREAM(rclcpp::get_logger("rclcpp"), "No type found in noise model. Ignoring noise model!");
        }
    }

    // ros_node_ = gazebo_ros::Node::Get(sdf, parent);
    ros_node_ = gazebo_ros::Node::Get(sdf);
    transform_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(ros_node_);
    const gazebo_ros::QoS& qos = ros_node_->get_qos();

    vel_sub_ = ros_node_->create_subscription<geometry_msgs::msg::Twist>(
        command_topic_, qos.get_subscription_qos(command_topic_, rclcpp::QoS(1)),
        std::bind(&PlanarMove::cmdVelCallback, this, std::placeholders::_1));
    odometry_pub_ = ros_node_->create_publisher<nav_msgs::msg::Odometry>(odometry_topic_,
                                                                         qos.get_publisher_qos("odom", rclcpp::QoS(1)));
    imu_pub_ = ros_node_->create_publisher<sensor_msgs::msg::Imu>("imu", qos.get_publisher_qos("imu", rclcpp::QoS(1)));

    // listen to the update event (broadcast every simulation iteration)
    last_update_time_ = model_->GetWorld()->SimTime().Double();
    last_publish_time_ = model_->GetWorld()->SimTime().Double();
    update_connection_ = event::Events::ConnectBeforePhysicsUpdate(std::bind(&PlanarMove::UpdateChild, this));

    links_list_ = model_->GetLinks();
    base_link_ = model_->GetLink(robot_base_frame_);
}

void PlanarMove::UpdateChild()
{
    //    // block any other physics pose updates
    //    boost::recursive_mutex::scoped_lock plock(*model_->GetWorld()->Physics()->GetPhysicsUpdateMutex());
    //    RCLCPP_WARN_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 250,
    //                                "UpdateChild()");
    CmdVel last_cmd;
    bool new_cmd_cp;
    {
        //        RCLCPP_WARN_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 250,
        //                                    "GZ WAITING LOCK 140");
        std::unique_lock<std::mutex> lock(cmd_lock);
        last_cmd = cmd_;
        new_cmd_cp = new_cmd_;
    }
    //    RCLCPP_WARN_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 250,
    //                                "GZ WAITING LOCK RELEASE 140");
    double gz_time_now = model_->GetWorld()->SimTime().Double();
    const bool is_paused = model_->GetWorld()->IsPaused();
    tf2::Quaternion tracked_qt;
    tracked_qt.setRPY(0, 0, tracked_state_.w);

    const double dt_since_last_publish = gz_time_now - last_publish_time_;
    if (dt_since_last_publish >= publish_period_)
    {
        const rclcpp::Time current_time = gazebo_ros::Convert<builtin_interfaces::msg::Time>(gz_time_now);
        if (publish_tf_) {
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
            //        RCLCPP_WARN_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 250,
            //                                    "sendTransform()");
        }

        if (publish_odometry_) {
            nav_msgs::msg::Odometry odom;

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

            if (control_mode_ == "position") {
                odom.twist.twist.linear.x = last_cmd.x;
                odom.twist.twist.linear.y = last_cmd.y;
                odom.twist.twist.linear.z = 0;
                odom.twist.twist.angular.x = 0;
                odom.twist.twist.angular.y = 0;
                odom.twist.twist.angular.z = last_cmd.w;
            } else {
                ignition::math::Vector3d linear_velocity = model_->RelativeLinearVel();
                odom.twist.twist.linear.x = linear_velocity.X();
                odom.twist.twist.linear.y = linear_velocity.Y();
                odom.twist.twist.linear.z = linear_velocity.Z();
                ignition::math::Vector3d rot_velocity = model_->RelativeAngularVel();
                odom.twist.twist.angular.x = rot_velocity.X();
                odom.twist.twist.angular.y = rot_velocity.Y();
                odom.twist.twist.angular.z = rot_velocity.Z();
            }

            odom.header.stamp = current_time;
            odom.header.frame_id = odometry_frame_;
            odom.child_frame_id = robot_base_frame_;

            odometry_pub_->publish(odom);
            //        RCLCPP_WARN_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 250,
            //                                    "publish odom()");
        }

        if (publish_imu_) {
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

            imu.angular_velocity.x = 0;
            imu.angular_velocity.y = 0;
            imu.angular_velocity.z = last_cmd.w;

            imu.angular_velocity_covariance[0] = 0.0001;
            imu.angular_velocity_covariance[4] = 0.0001;
            imu.angular_velocity_covariance[8] = 0.0001;

            imu.linear_acceleration.x = 0.00;
            imu.linear_acceleration.y = 0.00;
            imu.linear_acceleration.z = 9.81;

        }
    }
    // Update Loop
    const double dt_since_last_update = gz_time_now - last_update_time_;
    // RCLCPP_WARN_STREAM_THROTTLE(ros_node_->get_logger(), *ros_node_->get_clock(), 100,
    //                                "Sim time is: " << gz_time_now << "Last update dt is:" << dt_since_last_update <<
    //                                " wait for: " << 1.0/update_rate_);
    if (dt_since_last_update >= update_period_)
    {
        // Position control mode
        if (control_mode_ == "position")
        {
            RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Updating gazebo model position");
            const ignition::math::Pose3d current_pose = model_->WorldPose();
            double current_yaw = current_pose.Rot().Yaw();

            // determine shift in position with perfect velocity tracking
            const double dx = dt_since_last_update * last_cmd.x * cos(current_yaw) -
                              dt_since_last_update * last_cmd.y * cos(M_PI / 2 - current_yaw);
            const double dy = dt_since_last_update * last_cmd.x * sin(current_yaw) +
                              dt_since_last_update * last_cmd.y * sin(M_PI / 2 - current_yaw);
            const double dw = dt_since_last_update * last_cmd.w;

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
                auto x_error = dx * drift_x;
                auto y_error = dy * drift_y;
                auto w_error = dw * drift_w;

                // add odom sensor noise
                if (dist_)
                {
                    x_error *= dist_->x(generator_);
                    y_error *= dist_->y(generator_);
                    w_error *= dist_->w(generator_);
                }

                tracked_state_.x += x_error + dx;
                tracked_state_.y += y_error + dy;
                tracked_state_.w += w_error + dw;
            }

            if (base_link_ == nullptr)
            {
                RCLCPP_FATAL_STREAM(rclcpp::get_logger("rclcpp"), "Model has no link named 'base link'");
            }
            else
            {
                model_->GetWorld()->SetPaused(true);
                model_->SetLinkWorldPose(new_pose, base_link_);
                model_->GetWorld()->SetPaused(is_paused);
            }
        }
        // Velocity control mode
        else if (control_mode_ == "velocity")
        {
            if (new_cmd_cp)
            {
                RCLCPP_DEBUG_STREAM(ros_node_->get_logger(), "Updating gazebo model velocity");
                ignition::math::Pose3d pose = model_->WorldPose();
                const double yaw = pose.Rot().Yaw();
                model_->SetLinearVel(ignition::math::Vector3d(last_cmd.x * cos(yaw) - last_cmd.y * sin(yaw),
                                                              last_cmd.y * cos(yaw) + last_cmd.x * sin(yaw), 0));
                model_->SetAngularVel(ignition::math::Vector3d(0, 0, last_cmd.w));
                //                new_cmd_cp = false;
                {
                    std::unique_lock<std::mutex> lock(cmd_lock);
                    new_cmd_ = false;
                }
            }
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
    // Note there is no mechanism to zero cmd_vel's. move_base or cmd_vel mux should send 0

    {
        std::unique_lock<std::mutex> lock(cmd_lock);
        new_cmd_ = true;
        cmd_ = {cmd_msg.linear.x, cmd_msg.linear.y, cmd_msg.angular.z};
    }
}

GZ_REGISTER_MODEL_PLUGIN(PlanarMove)
}  // namespace gazebo
