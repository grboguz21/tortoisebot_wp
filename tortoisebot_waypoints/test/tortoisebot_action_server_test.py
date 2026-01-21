#!/usr/bin/env python
import rospy
import rostest
import unittest
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion
import actionlib
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal
from actionlib_msgs.msg import GoalStatus

PKG = 'tortoisebot_waypoints'
NAME = 'tortoisebot_action_server_test'


class TestWaypoint(unittest.TestCase):

    def setUp(self):
        rospy.init_node('test_waypoint')

        # ---------------- STATE ----------------
        self.current_position = Point()
        self.current_orientation = Quaternion()
        self.scan = None

        # ---------------- SUBSCRIBERS ----------------
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.loginfo("Waiting for odom and scan...")
        rospy.wait_for_message('/odom', Odometry, timeout=5)
        rospy.wait_for_message('/scan', LaserScan, timeout=5)

        # ---------------- START POSITION ----------------
        self.start_position = Point()
        self.start_position.x = self.current_position.x
        self.start_position.y = self.current_position.y
        self.start_position.z = self.current_position.z




        # ---------------- CONDITIONS ----------------
        self.target_position = Point()
        self.target_position.x = 0.5
        self.target_position.y = 0.5
        self.target_position.z = 0.0





        # ---------------- TOLERANCES ----------------
        self.pos_tolerance = 0.05  # meters
        self.yaw_tolerance = 0.05  # rad
        self.laser_limit = 0.100     # meters

    # --------------------------------------------------
    # CALLBACKS
    # --------------------------------------------------
    def odom_callback(self, msg):
        self.current_position = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation

    def scan_callback(self, msg):
        self.scan = msg

    # --------------------------------------------------
    # HELPERS
    # --------------------------------------------------
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
        return client

    # --------------------------------------------------
    # TESTS
    # --------------------------------------------------
    def test_position(self):
        client = self.send_goal()
        client.wait_for_result(rospy.Duration(20))  # max 20s

        err_pos = math.sqrt(
            (self.target_position.x - self.current_position.x) ** 2 +
            (self.target_position.y - self.current_position.y) ** 2
        )
        rospy.logwarn("[TEST] Position error: %.3f m", err_pos)
        self.assertTrue(err_pos < self.pos_tolerance,
                        "Position error too large: {:.3f}".format(err_pos))

    def test_rotation(self):
        client = self.send_goal()
        client.wait_for_result(rospy.Duration(20))  # max 20s

        final_yaw = self.get_yaw(self.current_orientation)
        desired_yaw = math.atan2(
            self.target_position.y - self.start_position.y,
            self.target_position.x - self.start_position.x
        )

        yaw_error = final_yaw - desired_yaw
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))
        yaw_error = abs(yaw_error)

        rospy.logwarn("[TEST] YAW -> final=%.3f | desired=%.3f | error=%.3f",
                      final_yaw, desired_yaw, yaw_error)
        self.assertTrue(yaw_error < self.yaw_tolerance,
                        "Yaw error too large: {:.3f}".format(yaw_error))

    def test_obstacle(self):
        client = self.send_goal()

        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(20)  # max 20s

        while not rospy.is_shutdown():
            # Action durumu
            state = client.get_state()
            if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.PREEMPTED]:
                break

            # Timeout kontrolü
            if rospy.Time.now() > timeout:
                client.cancel_goal()
                self.fail("Timeout: Robot did not reach goal in time")

            # Laser kontrolü
            if self.scan:
                for i, r in enumerate(self.scan.ranges):
                    if not math.isinf(r) and not math.isnan(r) and r < self.laser_limit:
                        client.cancel_goal()
                        self.fail("Obstacle too close! Laser index {} distance = {:.3f} m".format(i, r))

            rate.sleep()

        rospy.loginfo("Laser test passed, no obstacle closer than {:.2f} m.".format(self.laser_limit))


# ------------------------------------------------------
if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypoint)
