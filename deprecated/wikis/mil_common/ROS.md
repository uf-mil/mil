This is an incomplete page detailing some useful elements of ROS.

# Rospy

rospy.wait_for_message: Creates a new subscription to a topic, waits for a single message, and then unsubscribes.

# gtest

catkin_make will automatically build the unit tests in your package's "test" directory. Run your gtests with:
>      catkin_make run_tests_packageName
>      catkin_make run_tests_packageName_gtest_testTarget

Run all unit tests with: 
>      catkin_make run_tests