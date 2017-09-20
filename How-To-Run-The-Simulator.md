Launching the simulator will automatically generate the virtual world and bring the core systems of NaviGator online inside of it. The majority of the sensors (cameras, lidar, etc.) are implemented in simulation, so the platform should behave as it does in the real world for the most part.
* Run `roslaunch navigator_launch simulation.launch`
* Make sure alarms and lqRRT topics are active, if not: `roslaunch navigator_launch alarms.launch` and `rosrun navigator_path_planner lqrrt_node.py`
* To move around run `aclear all` and then use the alias `navc` to preform moves such as `navc f 1m`
* If you are running missions run `aclear all` then run `rosrun navigator_missions <name_of_mission>`