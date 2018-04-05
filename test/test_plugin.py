#!/usr/bin/env python

import unittest

import rospy
import rostest
from gazebo_msgs.srv import GetLinkState


class TestPlugin(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.__get_link_state_srv = rospy.ServiceProxy(
            name='/gazebo/get_link_state',
            service_class=GetLinkState
        )
        cls.__get_link_state_srv.wait_for_service(timeout=30)

    def test_calibrate(self):
        pass


if __name__ == '__main__':
    rospy.init_node('test_plugin')

    rostest.rosrun('gazebo_planar_move_plugin', 'test_plugin', TestPlugin)
