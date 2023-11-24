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

# License

Copyright 2023 The Boeing Company

Licensed under the Apache License, Version 2.0 (the "License") with the following modification;
you may not use this file except in compliance with the Apache License and the following modification to it:

(Appended as Section 10)

By accepting this software, recipient agrees that the representations, warranties, obligations, and liabilities of The Boeing Company set forth in this software, if any, are exclusive and in substitution for all other all other representations, warranties, obligations, and liabilities of The Boeing Company.
Recipient hereby waives, releases, and renounces all other rights, remedies, and claims (including tortious claims for loss of or damage to property) of recipient against The Boeing Company with respect to this software.
The Boeing Company hereby disclaims all implied warranties, including but not limited to, all implied warranties of merchantability, fitness, course of dealing, and usage of trade.
The Boeing Company shall have no liability, whether arising in contract (including warranty), tort (whether or not arising from the negligence of The Boeing Company), or otherwise, for any loss of use, revenue, or profit, or for any other unspecified direct, indirect, incidental, or consequential damages for anything delivered or otherwise provided by The Boeing Company under this software.

You may obtain a copy of the original, unmodified License at

   http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
echo 

# Contributing

Any contribution that you make to this repository will
be under the Modified Apache 2 License, as dictated by that
[license](http://www.apache.org/licenses/LICENSE-2.0):

```
5. Submission of Contributions. Unless You explicitly state otherwise,
   any Contribution intentionally submitted for inclusion in the Work
   by You to the Licensor shall be under the terms and conditions of
   this License, without any additional terms or conditions.
   Notwithstanding the above, nothing herein shall supersede or modify
   the terms of any separate license agreement you may have executed
   with Licensor regarding such Contributions.
```

To contribute, issue a PR and @brta-jc (jason.cochrane@boeing.com)
