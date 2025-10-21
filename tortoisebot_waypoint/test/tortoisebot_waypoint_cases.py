#! /usr/bin/env python

import rospy
import rosunit
import unittest
import rostest
from geometry_msgs.msg import Twist
import time

PKG = 'tortoisebot_waypoint'
NAME = 'tortoisebot_waypoint_cases'


class CaseA(unittest.TestCase):

    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        self.assert_(True)

class CaseB(unittest.TestCase):

    def setUp(self):
        self.rc = RobotControl()

    def runTest(self):
        self.assert_(True)
class MyTestSuite(unittest.TestSuite):

    def __init__(self):
        super(MyTestSuite, self).__init__()
        self.addTest(CaseA())
        self.addTest(CaseB())
