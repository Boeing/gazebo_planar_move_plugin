#!/usr/bin/env python

import unittest

import rospy
import rostest
from gazebo_msgs.srv import GetLinkState
from gazebo_msgs.srv import GetLinkStateRequest, GetLinkStateResponse
from geometry_msgs.msg import Twist


class TestPlugin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.__get_link_state_srv = rospy.ServiceProxy(
            name='/gazebo/get_link_state',
            service_class=GetLinkState
        )
        cls.__get_link_state_srv.wait_for_service(timeout=30)

        cls.__cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=100, latch=True)

        cls.__test_link_name = 'base_link'

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
        self.set_and_test_velocity(v_x=1.0,
                                   v_y=1.0,
                                   v_z=0.0,
                                   v_rz=0.5)

    def set_and_test_velocity(self, v_x, v_y, v_z, v_rz):
        # Set test velocity
        test_twist = self.make_twist(self, v_x, v_y, v_z, v_rz)
        self.__cmd_vel_pub.publish(test_twist)

        rospy.sleep(0.5)  # Small sleep to wait for the set to take effect

        # Get link velocity
        get_link_response = self.__get_link_state_srv.call(
            GetLinkStateRequest(link_name=self.__test_link_name,
                                reference_frame='world'))

        rospy.loginfo('Link state retrieved as value: ''{}'' \nexpected ''{}'''
                      .format(get_link_response.link_state.twist, test_twist))
        assert isinstance(get_link_response, GetLinkStateResponse)
        self.assertTrue(get_link_response.success)

        self.assertAlmostEqual(
            get_link_response.link_state.twist.linear.x, test_twist.linear.x, delta=0.01)
        self.assertAlmostEqual(
            get_link_response.link_state.twist.linear.y, test_twist.linear.y, delta=0.01)
        self.assertAlmostEqual(
            get_link_response.link_state.twist.linear.z, test_twist.linear.z, delta=0.01)
        self.assertAlmostEqual(
            get_link_response.link_state.twist.angular.z, test_twist.angular.z, delta=0.01)

        test_twist = self.make_twist(self, 0.0, 0.0, 0.0, 0.0)
        self.__cmd_vel_pub.publish(test_twist)

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


if __name__ == '__main__':
    rospy.init_node('test_plugin')

    rostest.rosrun('gazebo_planar_move_plugin', 'test_plugin', TestPlugin)
