#!/usr/bin/env python3
import preflightFileIO
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

    preflightFileIO.writeTests(
        "SubChecklist.txt",
        ["h check safety", "s TOPIC /dvl", "h check valve"],
    )
    preflightFileIO.deleteTests("SubChecklist.txt", ["check safety"])
    print(preflightFileIO.readTests("SubChecklist.txt"))
    print(preflightFileIO.runTests("SubChecklist.txt", 1))
