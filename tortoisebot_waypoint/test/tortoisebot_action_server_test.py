#! /usr/bin/env python3
import rospy
import unittest
import rostest
import actionlib
import math
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal
from actionlib_msgs.msg import GoalStatus
from tf.transformations import euler_from_quaternion

PKG = 'tortoisebot_waypoint'
NAME = 'tortoisebot_action_server_test'

def normalize_angle(rad):
    a = (rad + math.pi) % (2*math.pi)
    if a < 0:
        a += 2*math.pi
    return a - math.pi
def shortest_angle_diff_rad(target_rad, current_rad):
    d = (target_rad - current_rad + math.pi) % (2.0 * math.pi) - math.pi
    return d

class TestWaypointAction(unittest.TestCase):
    def setUp(self):
        rospy.init_node('test_waypoint_action', anonymous=True)
        self.client = actionlib.SimpleActionClient('/tortoisebot_as', WaypointActionAction)

        self.current_position = Point()
        self.current_yaw = None

        # Başlangıç ofsetleri
        self._yaw0 = None
        self._p0 = None

        # TEK HEDEF (her iki testte de kullanılacak)
        # launch'tan ~goal_x, ~goal_y geçebilirsin; default: (0.10, 0.0)
        self.goal_x = rospy.get_param('~goal_x', 0.10)
        self.goal_y = rospy.get_param('~goal_y', 0.00)
        self.goal_point = Point(x=self.goal_x, y=self.goal_y, z=0.0)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.sleep(1.0)

        connected = self.client.wait_for_server(rospy.Duration(10))
        self.assertTrue(connected, "Action server not available within timeout")

    def odom_callback(self, msg):
        # ham pozisyon ve oryantasyon
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        yaw = normalize_angle(yaw)

        # ilk ölçümde ofsetleri kilitle
        if self._yaw0 is None:
            self._yaw0 = yaw
        if self._p0 is None:
            self._p0 = Point(x=p.x, y=p.y, z=p.z)

        # relatif (başlangıca göre sıfırlanmış) değerler
        self.current_yaw = normalize_angle(yaw - self._yaw0)
        self.current_position = Point(
            x = p.x - self._p0.x,
            y = p.y - self._p0.y,
            z = 0.0
        )

    # --- Test 1: Action + hedefe yakınlık kontrolü ---
    def test_action_succeeds_and_reaches_goal(self):
        goal = WaypointActionGoal()
        goal.position = self.goal_point 

        rospy.loginfo("Sending goal: x=%.2f, y=%.2f", goal.position.x, goal.position.y)
        self.client.send_goal(goal)

        finished = self.client.wait_for_result(rospy.Duration(45))
        self.assertTrue(finished, "Action did not finish within timeout")

        state = self.client.get_state()
        result = self.client.get_result()
        self.assertIsNotNone(result, "No result returned by action server")
        self.assertEqual(state, GoalStatus.SUCCEEDED,
                         f"Action state was {state}, expected SUCCEEDED (3)")

        # Position margin check (≤ 9 cm)
        rospy.sleep(1.0)
        x, y = self.current_position.x, self.current_position.y
        dist_error = math.hypot(goal.position.x - x, goal.position.y - y)
        rospy.loginfo("Distance to goal: %.3f m", dist_error)

        # Her durumda sapmayı logla
        if 0.0 <= dist_error <= 0.090:
            rospy.loginfo("✅ PASS: Robot reached within %.3f m tolerance.", dist_error)
        else:
            rospy.logwarn("❌ FAIL: Robot deviated %.3f m from goal (allowed ≤ 0.09 m).", dist_error)

        self.assertTrue(0.0 <= dist_error <= 0.090,
                        f"Robot position error {dist_error:.3f} m not within [0.0, 0.09] m")

    # --- Test 2: desired_yaw sapması [min_deg°, max_deg°] aralığında mı? ---
    # --- Test 2: Robot yönü ile (goal_x, goal_y) doğrultusu arasındaki açı hatası eşik altına iner mi? ---
    def test_desired_yaw_error_in_range(self):
        """
        desired_yaw = atan2(goal - current_pos)
        error = shortest_angle_diff(desired_yaw, current_yaw)
        Hata belirli bir süre içinde [0, max_deg] aralığına inmeli.
        NOT: Ofset reset YOK; callback'in güncel /odom'dan beslediği self.current_* kullanılır.
        """
        max_deg        = rospy.get_param('~yaw_max_deg', 5.0)      # hedef eşik (örn. ≤5°)
        timeout_sec    = rospy.get_param('~yaw_timeout_sec', 5.0)  # toplam bekleme süresi
        check_rate_hz  = rospy.get_param('~yaw_check_hz', 20.0)    # kaç Hz ölçüm

        r = rospy.Rate(check_rate_hz)

        # Hedef (Test 1 ile aynı hedef)
        goal_x, goal_y = self.goal_x, self.goal_y

        start = rospy.Time.now()
        min_err = float('inf')

        while (rospy.Time.now() - start).to_sec() < timeout_sec and not rospy.is_shutdown():
            # En güncel poz/yaw (callback’ten gelir)
            p = self.current_position
            yaw = self.current_yaw
            if yaw is None:
                r.sleep()
                continue

            # Mevcut pozisyona göre hedef yönü
            desired_yaw = normalize_angle(math.atan2(goal_y - p.y, goal_x - p.x))

            # Kısa açı farkı
            err_rad = shortest_angle_diff_rad(desired_yaw, yaw)
            err_deg = abs(math.degrees(err_rad))
            min_err = min(min_err, err_deg)

            rospy.loginfo(
                "[YawCheck] goal=(%.3f, %.3f) pos=(%.3f, %.3f) desired=%.2f° current=%.2f° err=%.2f° (best=%.2f°)",
                goal_x, goal_y, p.x, p.y, math.degrees(desired_yaw), math.degrees(yaw), err_deg, min_err
            )

            if err_deg <= max_deg:
                rospy.loginfo("✅ PASS: Yaw error %.2f° <= %.2f°", err_deg, max_deg)
                break

            r.sleep()

        rospy.loginfo("---- Yaw final: min_err=%.2f°, threshold=%.2f°, timeout=%.2fs ----",
                    min_err, max_deg, timeout_sec)

        self.assertTrue(
            min_err <= max_deg,
            f"Yaw difference min {min_err:.2f}° did not fall within ≤ {max_deg:.2f}° in {timeout_sec:.1f}s"
        )

if __name__ == '__main__':
    rostest.rosrun(PKG, NAME, TestWaypointAction)
