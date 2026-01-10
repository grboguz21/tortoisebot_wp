#!/usr/bin/env python3
import rospy
import rostest
import unittest
import time
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion
import actionlib
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal

PKG = 'tortoisebot_waypoint'
NAME = 'tortoisebot_position_yaw_test'

class TestWaypointPositionYaw(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoint_position_yaw_checker')
        
        self.current_position = Point()
        self.current_orientation = Quaternion()
        rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.client = actionlib.SimpleActionClient('/tortoisebot_as', WaypointActionAction)
        rospy.loginfo("Waiting for action server...")
        self.client.wait_for_server()
        rospy.loginfo("Action server ready")

        # Hedef pozisyon
        # fail condition: Point(x=-1.0, y=0.2, z=0.0)
        # pass condition: Point(x=0.0, y=0.2, z=0.0)
        self.goal_position = Point(x=-1.0, y=0.2, z=0.0)

        # Gönder
        goal = WaypointActionGoal()
        goal.position = self.goal_position
        rospy.loginfo(f"Sending goal: x={goal.position.x}, y={goal.position.y}")
        self.client.send_goal(goal)
        self.client.wait_for_result() 
        
        rospy.sleep(1)
        self.final_position = self.current_position
        self.final_yaw = self.euler_to_yaw(self.current_orientation)
        rospy.loginfo(f"Final position: x={self.final_position.x}, y={self.final_position.y}")
        rospy.loginfo(f"Final yaw: {self.final_yaw}")

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def euler_to_yaw(self, quat_msg):
        orientation_list = [quat_msg.x, quat_msg.y, quat_msg.z, quat_msg.w]
        (_, _, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def test_reach_target_position(self):
        dx = self.goal_position.x - self.final_position.x
        dy = self.goal_position.y - self.final_position.y
        dist_error = math.sqrt(dx*dx + dy*dy)
        rospy.loginfo(f"Distance error: {dist_error}")
        self.assertTrue(
            dist_error <= 0.25,
            f"Robot did not reach the target position. Error = {dist_error} m"
        )

    def test_correct_yaw(self):
        init_yaw = 0.0   
        yaw_diff = abs(init_yaw - self.final_yaw)
        rospy.loginfo(f"Yaw diff: {yaw_diff}")
        self.assertTrue(
            1.3 <= abs(yaw_diff) <= 3.0,
            f"Integration error. Rotation was not between the expected values. Yaw Diff: {yaw_diff}"
        )

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointPositionYaw)
