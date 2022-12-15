# Gazebo Planar Movement Plugin

[![pipeline status](https://git.web.boeing.com/robotics/ros/gazebo_planar_move_plugin/badges/master/pipeline.svg)](https://git.web.boeing.com/robotics/ros/gazebo_planar_move_plugin/commits/master)
[![coverage](https://git.web.boeing.com/robotics/ros/gazebo_planar_move_plugin/badges/master/coverage.svg)](https://git.web.boeing.com/robotics/ros/gazebo_planar_move_plugin/commits/master)

## Motivation

Movement planning packages in ROS for differential drive or omni-directional drive mobile robots usually publish a _Twist_ message on the topic cmd_vel.
In order to simulate these robots, a Gazebo plugin is required to listen to this topic, then apply the appropriate movement to the simulated robot _Model_ in Gazebo.

## Goals

- If in _position_ mode:
  - Listen to the _/cmd_vel_ _Twist_ message and update the appropriate Gazebo _Model_ with a time interpolated position update based on the _Twist_ linear and angular velocity.
- If in _velocity_ mode:
  - Listen to the _/cmd_vel_ _Twist_ message and update the appropriate Gazebo _Model_ velocities with the _Twist_ linear and angular velocities.
- Odometry in the form of _nav_msgs/Odometry_ shall be published on /odom or _odometry_topic_ parmeter. This is usually used by external ROS localisation or navigation packages.
  - If _ground_truth_ is False, configurable noise shall be applied to the odometry value.
- Imu tracking shall be published in the form of _sensor_msgs/Imu_.

## Requirements

- Physics must be disabled using _gazebo_no_physics_plugin_ if in _position_ mode.

## Definitions

| Definition | Description                                                                                                              |
| ---------- | ------------------------------------------------------------------------------------------------------------------------ |
| Model       | An instanced SDF in Gazebo. _Models_ consist of _Links_ connected by _Joints_.                                                                                                               |

## Design

### Assumptions

### Limitations

- The plugin simulates planar movements only. Any Z axis components of velocity shall be ignored.
  - The model position Z component shall always be set to 0.0.
- Position uncertainty is not reflected in the _covariance_ components of the _Odom_ and _Imu_ topics published.
  - All relevant xyz rx ry rz covariance components are set to 0.0001.

## Requirements Evaluation

| Requirement | Met? | Comments |
| ------------| ------- | ---------- |
| Model responds to /cmd_vel and is moved in the XY plane in Gazebo. | Yes | None |
| Odometry is published and an appropriate noise model can be applied. | Yes | None |
| IMU is simulated and published. | Yes | None |

## Related Components

| Name                | Link                                                                       |
| ------------------- | -------------------------------------------------------------------------- |
| gazebo_ros_pkgs | [https://git.web.boeing.com/robotics/ros-thirdparty/gazebo_ros_pkgs](https://git.web.boeing.com/robotics/ros-thirdparty/gazebo_ros_pkgs) |
| gazebo_no_physics_plugin | [https://git.web.boeing.com/robotics/ros/gazebo_no_physics_plugin](https://git.web.boeing.com/robotics/ros/gazebo_no_physics_plugin) |
| modular_navigation | [https://git.web.boeing.com/brta-robotics/ros/modular_navigation](https://git.web.boeing.com/brta-robotics/ros/modular_navigation) |
| cartographer | [https://git.web.boeing.com/brta-robotics/cartographer](https://git.web.boeing.com/brta-robotics/cartographer) |