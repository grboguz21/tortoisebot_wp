#! /usr/bin/env python

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, Twist
from tf.transformations import euler_from_quaternion
import rospy

import rostest
import unittest
import time

PKG = 'tortoisebot_waypoint'
NAME = 'tortoisebot_action_server_test'

class TestTortoisebotActionServer(unittest.TestCase):

    def setUp(self):

        rospy.init_node('tortoisebot_action_server_test_node')
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.cmd_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        self.current_orientation = Quaternion()
        self.init_yaw = 0.0
        self.final_yaw = 0.0
        self.odom_ready = False

        rospy.wait_for_message("/odom", Odometry, timeout=10)
        self.init_yaw = self.euler_to_quaternion(self.current_orientation)
        rospy.loginfo("Initial yaw saved")


    def odom_callback(self, msg):
        self.current_orientation = msg.pose.pose.orientation
        self.odom_ready = True

    def euler_to_quaternion(self, msg):
        orientation_list = [msg.x, msg.y, msg.z, msg.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def spin_robot(self, angular_speed=0.6, duration=3.0):
        twist = Twist()
        twist.angular.z = angular_speed
        t0 = rospy.Time.now().to_sec()
        rate = rospy.Rate(20)
        while (rospy.Time.now().to_sec() - t0) < duration and not rospy.is_shutdown():
            self.cmd_pub.publish(twist)
            rate.sleep()
        self.cmd_pub.publish(Twist())
        rospy.sleep(0.5)

    def test_correct_rotation(self):
        rospy.loginfo("Starting rotation test")

        self.spin_robot(angular_speed=0.6, duration=3.0)

        # Dönüş sonrası yaw hesapla
        self.final_yaw = self.euler_to_quaternion(self.current_orientation)
        yaw_diff = abs(self.final_yaw - self.init_yaw)

        rospy.loginfo(f"Initial yaw: {self.init_yaw:.3f}")
        rospy.loginfo(f"Final yaw:   {self.final_yaw:.3f}")
        rospy.loginfo(f"Yaw diff:    {yaw_diff:.3f} rad")

        # ± aralık toleransı
        self.assertTrue(
            (1.0 <= yaw_diff <= 5.5),
            f"Rotation out of expected range! (yaw_diff={yaw_diff:.3f} rad)"
        )

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestTortoisebotActionServer)
