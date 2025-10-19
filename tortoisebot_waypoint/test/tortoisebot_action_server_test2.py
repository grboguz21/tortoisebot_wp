#! /usr/bin/env python3
import rospy
import rostest
import unittest
import math
import actionlib

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Quaternion
from tf.transformations import euler_from_quaternion
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal

PKG  = 'tortoisebot_waypoint'
NAME = 'tortoisebot_goal_checks_test'

def angdiff(a, b):
    """Yaw farkını (-pi, pi) aralığında döndürür."""
    return (a - b + math.pi) % (2.0 * math.pi) - math.pi

class TestWaypointGoalChecks(unittest.TestCase):

    def setUp(self):
        rospy.init_node('tortoisebot_goal_checks_test_node')

        # Toleranslar (param yoksa varsayılan)
        self.pos_tol   = rospy.get_param("~pos_tol", 0.10)     # metre
        self.yaw_tol_d = rospy.get_param("~yaw_tol_deg", 5.0)  # derece

        # Odom abone
        self.current_position    = Point()
        self.current_orientation = Quaternion()
        self.odom_ready = False
        rospy.Subscriber('/odom', Odometry, self.odom_callback, queue_size=10)

        # İlk /odom’u bekle
        got = rospy.wait_for_message("/odom", Odometry, timeout=10.0)
        self.current_position    = got.pose.pose.position
        self.current_orientation = got.pose.pose.orientation
        self.odom_ready = True

        # Action client
        self.client = actionlib.SimpleActionClient('tortoisebot_as', WaypointActionAction)
        ok = self.client.wait_for_server(rospy.Duration(10.0))
        self.assertTrue(ok, "Action server 'tortoisebot_as' gelmedi!")

    def odom_callback(self, msg):
        self.current_position    = msg.pose.pose.position
        self.current_orientation = msg.pose.pose.orientation
        self.odom_ready = True

    # Senin isimlendirme tarzınla (quaternion -> yaw):
    def euler_to_quaternion(self, q_msg: Quaternion):
        orientation_list = [q_msg.x, q_msg.y, q_msg.z, q_msg.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        return yaw

    def send_goal_and_wait(self, p: Point, timeout_s: float = 90.0):
        g = WaypointActionGoal()
        g.goal.position = p
        self.client.send_goal(g.goal)
        finished = self.client.wait_for_result(rospy.Duration(timeout_s))
        self.assertTrue(finished, "Action zaman aşımına uğradı ({} s)!".format(timeout_s))
        res = self.client.get_result()
        self.assertIsNotNone(res, "Action result None!")
        self.assertTrue(res.success, "Action success=False döndü!")
        rospy.sleep(0.5)  # dur-kal sonrası küçük bekleme

    # ---- Tek metotta hem konum hem yaw kontrolü ----
    def test_end_pose_and_yaw_are_expected(self):
        self.assertTrue(self.odom_ready, "/odom gelmedi!")

        # >>> Hedef: burayı istediğin gibi değiştir
        goal = Point(-0.2, 0.0, 0.0)   # örn: X ekseninde -0.2 m

        # Başlangıç konumuna göre beklenen yaw (bearing)
        x0 = self.current_position.x
        y0 = self.current_position.y
        desired_yaw = math.atan2(goal.y - y0, goal.x - x0)

        # Hedefe git
        self.send_goal_and_wait(goal)

        # --- Pozisyon kontrolü ---
        x = self.current_position.x
        y = self.current_position.y
        dist_err = math.hypot(goal.x - x, goal.y - y)
        rospy.loginfo(f"[POS] Hedef=({goal.x:.3f},{goal.y:.3f})  Son=({x:.3f},{y:.3f})  Hata={dist_err:.3f} m")
        self.assertLessEqual(
            dist_err, self.pos_tol,
            "Nihai konum toleransı aşıldı! dist_err={:.3f} m, tol={:.3f} m".format(dist_err, self.pos_tol)
        )

        # --- Yaw kontrolü ---
        final_yaw = self.euler_to_quaternion(self.current_orientation)
        yaw_err   = abs(angdiff(final_yaw, desired_yaw))
        yaw_err_d = math.degrees(yaw_err)
        rospy.loginfo(f"[YAW] Beklenen={desired_yaw:.3f} rad  Son={final_yaw:.3f} rad  Hata={yaw_err_d:.2f}°")
        self.assertLessEqual(
            yaw_err_d, self.yaw_tol_d,
            "Nihai yaw toleransı aşıldı! yaw_err={:.2f}°, tol={:.2f}°".format(yaw_err_d, self.yaw_tol_d)
        )

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointGoalChecks)
