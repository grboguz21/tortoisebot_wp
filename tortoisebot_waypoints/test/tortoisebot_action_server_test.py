#!/usr/bin/env python
import rospy
import rostest
import unittest
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion
import actionlib
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal

PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_action_server_test'

class TestWaypoint(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoint')

        self.current_position = Point()
        self.current_orientation = Quaternion()

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.loginfo("Waiting for odometry...")
        rospy.wait_for_message('/odom', Odometry, timeout=5)

        self.target_position = Point()
        self.target_position.x = 0.05
        self.target_position.y = 0.0
        self.target_position.z = 0.0

    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def get_yaw(self, q):
        quat_list = [q.x, q.y, q.z, q.w]
        (_, _, yaw) = euler_from_quaternion(quat_list)
        return yaw

    def send_goal(self):
        client = actionlib.SimpleActionClient('/tortoisebot_as', WaypointActionAction)
        rospy.loginfo("Waiting for action server...")
        client.wait_for_server()
        rospy.loginfo("Action server available")

        goal = WaypointActionGoal()
        goal.position = self.target_position

        rospy.loginfo("Sending goal: x=%.2f y=%.2f", goal.position.x, goal.position.y)
        client.send_goal(goal)
        client.wait_for_result()
        rospy.loginfo("Action server completed")

    def test_correct_rotation(self):
        # Goal gönder
        self.send_goal()

        # Final yaw
        final_yaw = self.get_yaw(self.current_orientation)
        rospy.loginfo("Final Yaw: %.3f", final_yaw)

        # --- Desired yaw hesapla ---
        desired_yaw = math.atan2(
            self.target_position.y - self.current_position.y,
            self.target_position.x - self.current_position.x
        )
        rospy.loginfo("Desired Yaw: %.3f", desired_yaw)

        # Yaw hatası
        yaw_error = abs(final_yaw - desired_yaw)
        rospy.loginfo("Yaw error: %.3f", yaw_error)
        print("Yaw error: {:.3f}".format(yaw_error))

        # ASSERT: Hata küçükse test geçecek
        self.assertTrue(
            yaw_error < 0.035,
            "Robot yaw error too large! Yaw error: {:.3f}".format(yaw_error)
        )

    def test_correct_position(self):
        # Goal gönder
        self.send_goal()

        # Euclidean distance ile hata hesapla
        err_pos = math.sqrt(
            pow(self.target_position.x - self.current_position.x, 2) +
            pow(self.target_position.y - self.current_position.y, 2)
        )

        rospy.loginfo("Current Position: x=%.3f y=%.3f", self.current_position.x, self.current_position.y)
        rospy.loginfo("Position error: %.3f", err_pos)
        print("Position error: {:.3f}".format(err_pos))

        tolerance = 0.05
        self.assertTrue(
            err_pos < tolerance,
            "Robot did not reach the target position within tolerance! Error: {:.3f}".format(err_pos)
        )


if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypoint)
