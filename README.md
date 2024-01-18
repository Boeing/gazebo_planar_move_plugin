# Gazebo Planar Movement Plugin
| Distro | CI Status                                                                                                                                                                                       |
| ------ | ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Humble | [![CI](https://github.com/Boeing/gazebo_planar_move_plugin/actions/workflows/main.yml/badge.svg?branch=humble)](https://github.com/Boeing/gazebo_planar_move_plugin/actions/workflows/main.yml) |

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

| Definition | Description                                                                    |
| ---------- | ------------------------------------------------------------------------------ |
| Model      | An instanced SDF in Gazebo. _Models_ consist of _Links_ connected by _Joints_. |

## Design

### Assumptions

### Limitations

- The plugin simulates planar movements only. Any Z axis components of velocity shall be ignored.
  - The model position Z component shall always be set to 0.0.
- Position uncertainty is not reflected in the _covariance_ components of the _Odom_ and _Imu_ topics published.
  - All relevant xyz rx ry rz covariance components are set to 0.0001.

# License

This package is released under the Apache 2.0 License

# Contributing

Any contribution that you make to this repository will
be under the Apache-2.0 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0)

To contribute, issue a PR and @brta-jc (jason.cochrane@boeing.com)
