# Overview
A ["bag"](http://wiki.ros.org/Bags) is a file format that stores a history or messages from one or more topics along with the time they were received. These files can be played back to publish the same messages in the same order and timing to recreate past scenario. This is extremely useful for testing code, as it allows the developer to run their programs with the same sort of data that is produced on the live system without requiring physical access to the system.

## Seeing bag content
Before playing a bag, you should make sure it actually has the topics you need to test your code. To do this, run:

`rosbag info <bagfile>`


## Time synchronization
By default, ROS will use your computer's current time in nodes. Often it is desirable or necessary to synchronize running nodes' time with the time from the bag playing back. To do this, first set the global parameter to use simulated time:

`rosparam set /use_sim_time True`

Any nodes that were previously running will need to be restarted after this param is changed.
Next, play the bag with the --clock flag to simulate the timestamps from the bag as it is played back.

`rosbag play <bag file> --clock`