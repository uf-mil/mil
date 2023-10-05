#!/usr/bin/env python3
import rospy

# Use Rich for nice printing
if __name__ == "__main__":
    # Initialize the Preflight Node
    rospy.init_node("preflight")
    print("Checking for Gazebo")
    # Check if the Gazebo Node exists
    if rospy.get_param("/is_simulation", None):
        print("Running a Simulation")
    else:
        print("Running for real")
