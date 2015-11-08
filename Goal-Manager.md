I have added to the Sub8 repo thus far half of the goal manager and some math tools that assist with common functions of operating the boat. This is still very much in the making but here is a good first start. 

# **What is the goal manager?:**

The goal manager will be used to control upper level move functions of the boat.
It is what will be used to send the waypoint locations that the boat will plan a path
to and arrive at. 

# **How the goal manager will work?**:

The client will take input from a called function and create a Odometry message
It will send it the server where it will send the goal to the path planner and then 
stall until either it has reached the set timeout or reached its desired location. 

So when calling a function from the goal_client, it will pause your program until either
a timeout has been reached or you reach the goal. This is how we will send multiple 
goals one after another. 

### How to test for the moment:

Run:

    rosrun goal_manager goal_server.py
    rosrun goal_manager test_goals.py

    