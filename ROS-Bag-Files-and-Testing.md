# **ROS Bag Files and Testing**

### Available Bag Files:
* Lidar Bouy Field
* Camera Bouy Field
* Camera Sign - Cross

There is a script located in the /utils/templates directory of the repo that has the basics for analyzing images. It subscribes to the ROS image topic and coverts it to an OpenCV datatype leaving the annoying part done and the fun part (analyzing the images) up to you!

[**CV Testing Script Link**](https://github.com/uf-mil/Navigator/blob/master/utils/templates/cv_testing)

### Running

Run in one terminal:

    rosbag play /path/to/file/name_of_bag_file.bag 

In another terminal navigate to the directory of the cv_testing file and run:

    ./cv_testing

You should see the video pop up and play as long as the cv testing script is running and a bag file with the /camera/image_raw file is playing. Now do some wizardry and track some objects!

You can download one bag file from [HERE](https://drive.google.com/open?id=0B-z-0c8O0GlpOHpaUmcwdTgzWEE) for now  to get started and I will get all of them uploaded to the server in MIL this week with instructions on how to use them. 

#### Lidar (Pointcloud) version to come

