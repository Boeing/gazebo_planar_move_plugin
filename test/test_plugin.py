import os
import unittest
import launch
import launch.actions

import launch_testing
import launch_testing.actions
from ament_index_python import get_package_share_directory

import subprocess
import rclpy
import rclpy.clock
import rclpy.time
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
import pytest
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from threading import Thread
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

# from gazebo_model_attachment_plugin.gazebo_client import GazeboModelAttachmentClient

from time import sleep


@pytest.mark.launch_test
def generate_test_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    world_file_name = os.path.join(get_package_share_directory('gazebo_planar_move_plugin'),
                                   'test', 'test.world')
    urdf_file_name = os.path.join(get_package_share_directory('gazebo_planar_move_plugin'),
                                  'test', 'test.urdf')

    print('robot  urdf_file_name : {}'.format(urdf_file_name))
    print('world world_file_name : {}'.format(world_file_name))

    return launch.LaunchDescription(
        [
            # Launch GAZEBO
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzserver.launch.py')
                ),
                launch_arguments={
                    'world': world_file_name, 'gui': '0'}.items(),
            ),

            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch',
                                 'gzclient.launch.py')
                ),
                launch_arguments={'gui': '0'}.items(),
            ),

            # Launch robot_state_publisher
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[{'use_sim_time': True}],
                arguments=[urdf_file_name]
            ),

            # Spawn robot in Gazebo
            Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-entity', 'test_robot', '-file', urdf_file_name],
                 output='screen'),

            launch_testing.actions.ReadyToTest(),
        ]
    )


# There is a bug where the gzserver is not killed after the tests run.
# https://github.com/ros2/launch/issues/545
# The issue should be fixed by https://github.com/ros-simulation/gazebo_ros_pkgs/pull/1376
# This is a workaround to kill the gzserver after the tests run.
# Remove this once gazebo updates the apt package to the latest version.
@launch_testing.post_shutdown_test()
class TestShutdown(unittest.TestCase):
    def test_kill_sim(self):
        subprocess.run(["pkill", "gzserver"])
        subprocess.run(["pkill", "gzclient"])


# These tests will run concurrently with the dut process.  After all these tests are done,
# the launch system will shut down the processes that it started up
class TestPlanarMovePlugin(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def odom_callback(self, msg):
        self.odom_twist = msg.twist.twist

    def setUp(self):
        def spin_srv(executor):
            try:
                executor.spin()
            except rclpy.executors.ExternalShutdownException:
                pass

        self.odom_twist = self.make_twist(self, 0.0, 0.0, 0.0, 0.0)
        self.node = rclpy.create_node('test_node', parameter_overrides=[
            Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        srv_executor = MultiThreadedExecutor()
        srv_executor.add_node(self.node)
        srv_thread = Thread(target=spin_srv, args=(srv_executor,), daemon=True)
        srv_thread.start()

        self.clock = rclpy.clock.Clock(
            clock_type=rclpy.clock.ClockType.ROS_TIME)
        self.log = self.node.get_logger()

        # Client for checking entity status in gazebo
        # self.log.info("Creating subscriber for joint state...")
        # self.__joint_state = None
        # self.__joint_state_sub = self.node.create_subscription(
        #     JointState,
        #     '/joint_states',
        #     self.configs_callback,
        #     10)

        # Twist publisher with latching QoS
        qos_profile = QoSProfile(
            depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)
        self.twist_publisher = self.node.create_publisher(
            Twist, 'cmd_vel', qos_profile)

        self.robot_name = 'test_robot'
        self.world_frame = 'world'

        self.odom_subscriber = self.node.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile=QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST))

        sleep(5)  # Give time to Gazebo client/server to bring up

    def tearDown(self):
        self.node.destroy_node()

    def set_and_test_velocity(self, v_x, v_y, v_z, v_rz):
        # Set test velocity
        test_twist = self.make_twist(self, v_x, v_y, v_z, v_rz)
        self.twist_publisher.publish(test_twist)

        sleep(5)  # Small sleep to wait for the set to take effect
        self.node.get_logger().info('Entity state retrieved as value: ''{}'' \nexpected ''{}'''
                                    .format(self.odom_twist, test_twist))

        self.assertAlmostEqual(
            self.odom_twist.linear.x, test_twist.linear.x, delta=0.01)
        self.assertAlmostEqual(
            self.odom_twist.linear.y, test_twist.linear.y, delta=0.01)
        self.assertAlmostEqual(
            self.odom_twist.linear.z, test_twist.linear.z, delta=0.01)
        self.assertAlmostEqual(
            self.odom_twist.angular.z, test_twist.angular.z, delta=0.01)

        test_twist = self.make_twist(self, 0.0, 0.0, 0.0, 0.0)
        self.twist_publisher.publish(test_twist)

    @staticmethod
    def make_twist(self, x, y, z, rz):
        test_twist = Twist()
        test_twist.linear.x = x
        test_twist.linear.y = y
        test_twist.linear.z = z
        test_twist.angular.x = 0.0
        test_twist.angular.y = 0.0
        test_twist.angular.z = rz
        return test_twist

    def test_set(self):
        # Test linear moves
        self.set_and_test_velocity(v_x=1.0,
                                   v_y=0.0,
                                   v_z=0.0,
                                   v_rz=0.0)

        self.set_and_test_velocity(v_x=-1.0,
                                   v_y=0.0,
                                   v_z=0.0,
                                   v_rz=0.0)

        self.set_and_test_velocity(v_x=0.0,
                                   v_y=1.0,
                                   v_z=0.0,
                                   v_rz=0.0)

        self.set_and_test_velocity(v_x=0.0,
                                   v_y=-1.0,
                                   v_z=0.0,
                                   v_rz=0.0)

        # Test rotation
        self.set_and_test_velocity(v_x=0.0,
                                   v_y=0.0,
                                   v_z=0.0,
                                   v_rz=0.5)

        self.set_and_test_velocity(v_x=0.0,
                                   v_y=0.0,
                                   v_z=0.0,
                                   v_rz=-0.5)

        # Test combo
        # This test is not valid - sin and cosine compnenets of YAW are applied to X and Y:
        # const double dx = dt * cmd_vel.linear.x * cos(yaw) - dt * cmd_vel.linear.y * cos(M_PI / 2 - yaw);
        # const double dy = dt * cmd_vel.linear.x * sin(yaw) + dt * cmd_vel.linear.y * sin(M_PI / 2 - yaw);
        # const double dw = dt * cmd_vel.angular.z;
        #
        # v_x = 1.0
        #
        # v_y = 1.0
        # v_z = 0.0
        # v_rz = 0.5
