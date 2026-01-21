#!/usr/bin/env python

import rospy
import actionlib
import math

from tortoisebot_msgs.msg import (
    WaypointActionAction,
    WaypointActionFeedback,
    WaypointActionResult
)

from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf import transformations


class WaypointActionClass(object):

    def __init__(self):

        # ---------------- ACTION SERVER ----------------
        self._as = actionlib.SimpleActionServer(
            "tortoisebot_as",
            WaypointActionAction,
            self.goal_callback,
            False
        )
        self._as.start()

        # ---------------- PUBLISHERS / SUBSCRIBERS ----------------
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._sub_odom = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self._sub_scan = rospy.Subscriber('/scan', LaserScan, self._scan_callback)

        # ---------------- RATE ----------------
        self._rate = rospy.Rate(25)

        # ---------------- ROBOT STATE ----------------
        self._position = Point()
        self._yaw = 0.0
        self._scan = None

        # ---------------- GOAL ----------------
        self._des_pos = Point()

        # ---------------- PRECISION ----------------
        self._dist_precision = 0.05          # 5 cm
        self._yaw_precision = math.pi / 90   # ~2 deg

        # ---------------- CONTROL GAINS ----------------
        self._kp_lin = 0.8
        self._kp_ang = 1.5

        # ---------------- SPEED LIMITS ----------------
        self._max_lin = 0.4
        self._max_ang = 0.8

        # ---------------- FEEDBACK / RESULT ----------------
        self._feedback = WaypointActionFeedback()
        self._result = WaypointActionResult()

        rospy.loginfo("✅ Tortoisebot Action Server started")

    # ----------------------------------------------------------
    # ---------------- CALLBACKS -------------------------------
    # ----------------------------------------------------------

    def _odom_callback(self, msg):
        self._position = msg.pose.pose.position
        q = msg.pose.pose.orientation
        quat = (q.x, q.y, q.z, q.w)
        (_, _, self._yaw) = transformations.euler_from_quaternion(quat)

    def _scan_callback(self, msg):
        self._scan = msg

    # ----------------------------------------------------------
    # ---------------- HELPER FUNCTIONS ------------------------
    # ----------------------------------------------------------

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_front_min_distance(self, fov_deg=20):
        if self._scan is None:
            return None
        half_fov = math.radians(fov_deg / 2.0)
        min_dist = float('inf')
        for i, r in enumerate(self._scan.ranges):
            angle = self._scan.angle_min + i * self._scan.angle_increment
            if -half_fov <= angle <= half_fov:
                if not math.isinf(r) and not math.isnan(r):
                    min_dist = min(min_dist, r)
        if min_dist == float('inf'):
            return None
        return min_dist

    # ----------------------------------------------------------
    # ---------------- GOAL CALLBACK ---------------------------
    # ----------------------------------------------------------

    def goal_callback(self, goal):

        rospy.loginfo("[INFO] New goal received")
        self._des_pos = goal.position
        success = True

        # ---------------- MOVE TO GOAL ----------------
        while not rospy.is_shutdown():

            if self._as.is_preempt_requested():
                rospy.loginfo("[INFO] Goal preempted")
                self._as.set_preempted()
                success = False
                break

            # ---------------- LASER SAFETY ----------------
            front_dist = self.get_front_min_distance()
            if front_dist is not None and front_dist < 0.100:
                rospy.logerr("[SAFETY] Obstacle too close! Stopping robot")
                self._pub_cmd_vel.publish(Twist())
                self._as.set_aborted()
                success = False
                break

            # Distance & Yaw
            err_pos = math.sqrt(
                (self._des_pos.x - self._position.x) ** 2 +
                (self._des_pos.y - self._position.y) ** 2
            )
            desired_yaw = math.atan2(
                self._des_pos.y - self._position.y,
                self._des_pos.x - self._position.x
            )
            err_yaw = self.normalize_angle(desired_yaw - self._yaw)

            # ---------------- CONTROL ----------------
            twist = Twist()
            ang_speed = max(min(self._kp_ang * err_yaw, self._max_ang), -self._max_ang)
            lin_speed = self._kp_lin * err_pos if abs(err_yaw) < math.pi / 6 else 0.0
            lin_speed = min(lin_speed, self._max_lin)

            twist.linear.x = lin_speed
            twist.angular.z = ang_speed
            self._pub_cmd_vel.publish(twist)

            # ---------------- FEEDBACK ----------------
            self._feedback.position = self._position
            self._feedback.state = "moving"
            self._as.publish_feedback(self._feedback)

            # Goal check
            if err_pos < self._dist_precision:
                break

            self._rate.sleep()

        # ---------------- FINAL YAW ALIGN ----------------
        if success:
            while not rospy.is_shutdown():
                target_yaw = math.atan2(
                    self._des_pos.y - self._position.y,
                    self._des_pos.x - self._position.x
                )
                err_yaw = self.normalize_angle(target_yaw - self._yaw)

                # log yaw error
                rospy.loginfo("[FINAL YAW ALIGN] target=%.3f | current=%.3f | error=%.3f",
                            target_yaw, self._yaw, err_yaw)

                if abs(err_yaw) < self._yaw_precision:
                    break
                twist = Twist()
                twist.angular.z = -max(min(self._kp_ang * err_yaw, self._max_ang), -self._max_ang)
                self._pub_cmd_vel.publish(twist)
                self._rate.sleep()

            # stop the robot
            self._pub_cmd_vel.publish(Twist())

            # terminal status set
            self._result.success = True
            self._as.set_succeeded(self._result)
            rospy.loginfo("[INFO] Goal reached successfully")




# ----------------------------------------------------------
if __name__ == '__main__':
    rospy.init_node('tortoisebot_as')
    WaypointActionClass()
    rospy.spin()
