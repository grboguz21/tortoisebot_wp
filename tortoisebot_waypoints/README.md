TASK 1 
------------------
  command: rostest tortoisebot_waypoint waypoints_test.test --reuse-master

  (tortoisebot_action_server_position_test.py in 32-33)
  -----------------------------------------------------
    # failing condition: Point(x=-1.0, y=0.2, z=0.0)
    # passing condition: Point(x=0.0, y=0.2, z=0.0)


-------0--------0--------0--------0--------0--------0--------0--------0--------0-------

PASSING CONDITION
------------------
user:~/simulation_ws$ rostopic pub -1 /tortoisebot_as/goal tortoisebot_msgs/WaypointActio
nActionGoal "goal:
  position:
    x: 0.05
    y: 0.0
    z: 0.0
"

FAILLING CONDITION (yaw error, dist pass)
------------------
user:~/simulation_ws$ rostopic pub -1 /tortoisebot_as/goal tortoisebot_msgs/WaypointActionActionGoal "
goal:
  position:
    x: 0.2
    y: 0.2
    z: 0.0
"


TASK 2
------------------
