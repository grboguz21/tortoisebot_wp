#! /usr/bin/env python
import rospy, actionlib
from geometry_msgs.msg import Point
from tortoisebot_msgs.msg import WaypointActionAction, WaypointActionGoal

rospy.init_node('tortoisebot_action_client')
client = actionlib.SimpleActionClient('/tortoisebot_as', WaypointActionAction)
client.wait_for_server()

goal = WaypointActionGoal()
goal.position = Point(x=0.2, y=0.2, z=0.0)

rospy.loginfo("Sending goal: x=%.2f y=%.2f", goal.position.x, goal.position.y)
client.send_goal(goal)
client.wait_for_result()
rospy.loginfo("Result: %s", client.get_result())
