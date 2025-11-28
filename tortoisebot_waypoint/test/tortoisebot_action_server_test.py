#!/usr/bin/env python3
import rospy
import rostest
import unittest
import time
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Point
from tf.transformations import euler_from_quaternion
import actionlib
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal

PKG = 'tortoisebot_waypoint'
NAME = 'tortoisebot_action_server_test'

class TestWaypointAction(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoint_action')
        
        # Action client
        self.client = actionlib.SimpleActionClient('/tortoisebot_as', WaypointActionAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server available")
        
        # Odometry subscriber
        self.current_orientation = Quaternion()
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.init_yaw = 0
        self.final_yaw = 0

    def odom_callback(self, msg):
        self.current_orientation = msg.pose.pose.orientation

    def euler_to_quaternion(self, msg):
        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def test_correct_rotation(self):
        print ("Current Orientation:")
        print(self.current_orientation)
        time.sleep(15)
        #rospy.wait_for_message("/odom", Odometry, timeout=2)
        self.final_yaw = self.euler_to_quaternion(self.current_orientation)
        print("Final Yaw:")
        print(self.final_yaw)
        yaw_diff = self.init_yaw - self.final_yaw
        print("Yaw Diff:")
        print(yaw_diff)
        self.assertTrue(
            (1.3 <= abs(yaw_diff) <= 3),
            "Integration error. Rotation was not between the expected values. Yaw Diff: {}".format(yaw_diff)
        )

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointAction)
