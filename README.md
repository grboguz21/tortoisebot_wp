TASK 1 
------------------

# PASSING CONDITION
In order to test the passing condition, navigate to: 
/home/user/simulation_ws/src/tortoisebot_waypoints/test/tortoisebot_action_server_test.py

replace the lines (26-29) with the following code:
    self.target_position = Point()
    self.target_position.x = 0.2
    self.target_position.y = 0.0
    self.target_position.z = 0.0


FAILLING CONDITION 
------------------
In order to test failing condition, navigate to: 
/home/user/simulation_ws/src/tortoisebot_waypoints/test/tortoisebot_action_server_test.py

replace the lines (26-29) with the following code:
    self.target_position = Point()
    self.target_position.x = 0.5
    self.target_position.y = 0.5
    self.target_position.z = 0.0
