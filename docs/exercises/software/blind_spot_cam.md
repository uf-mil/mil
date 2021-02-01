# DIY Blind Spot / Backup Camera

In this exercise, you will use a usb camera, an inexpensive android phone, and ROS to build a backup / blind spot camera for your car (or really any vehicle you like).


## Materials

```eval_rst
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
|     Item               | Cost  | Purchase Link                                                                                                                                                                                                                                                     |
+========================+=======+===================================================================================================================================================================================================================================================================+
| Used Android Phone     | ~$50  | `Ebay <https://www.ebay.com/sch/i.html?_from=R40&_nkw=used+android+Phone&_sacat=0&_udlo=30&_udhi=60&rt=nc&LH_BIN=1>`_                                                                                                                                             |
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Wide Angle Usb Web Cam | ~$10  | `Amazon <https://www.amazon.com/gp/product/B08HQ4773T/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1>`_                                                                                                                                                         |
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| USB C to USB A Female  | ~$9   | `For USB C Phones <https://www.amazon.com/AmazonBasics-Type-C-Gen1-Female-Adapter/dp/B01GGKYXVE/ref=sr_1_4?dchild=1&keywords=usb+C+to+usb+a+female&qid=1612144556&refinements=p_85%3A2470955011%2Cp_36%3A1253503011&rnid=386442011&rps=1&s=electronics&sr=1-4>`__ |
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| OR                     |       |                                                                                                                                                                                                                                                                   |
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| Micro USB to USB A     | ~$5   | `For USB Micro B Phones <https://www.amazon.com/Ksmile%C2%AE-Female-Adapter-SamSung-tablets/dp/B01C6032G0/ref=sr_1_7?dchild=1&keywords=usb+micro+b+to+usb+a+female&qid=1612144669&s=electronics&sr=1-7>`__                                                        |
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
| USB A Extender         | ~$10  | `Amazon <https://www.amazon.com/gp/product/B07ZV6FHWF/ref=ppx_yo_dt_b_asin_title_o00_s00?ie=UTF8&psc=1>`__                                                                                                                                                        |
+------------------------+-------+-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------+
```

## Learning Goals For This Exercise

* How to use a remote linux computer (in the form of an android phone with a ubuntu virtual file system running ROS) via ssh and git.

* How to use `rostopic list/echo/hz`, `rqt_graph`, and `rviz` to invesitgate a ROS system without reading soruce code.

* How to use the MIL `ros_connect script`

* How to use the ROS Mobile App

* How to use rospy to subscribe and publish a topic

* How to use OpenCv2 to rotate an image

* How to use rosbag to collect data

## Setting Everything Up

In Mil, each robot is its own stand alone linux computer. These computers are accessed over the network, they are not interacted with via a screen / keyboard / mouse, like a personal computer.

A used android phone is one of the most cost effective linux computers availible once you consider that it is self powered, ruggedized, has a display, a touch screen, GPS, IMU, Camera, Wifi, and expandable storage.

### Setting up your personal computer

Complete the Onboarding guide

### Provisioning the Android Device

Unfortunately, while Android does use a slightly modified linux kernel, it has a completely different filesystem. However, we have found a solution to this. There exists a utility in Linux called `chroot` and it is at the heart of the android provisioning scripts.

`chroot` allows us to virtualize the ubnutu file system on the sdcard of the android device, which allows ROS to run on andorid device without re-virtualizing the android kernel.

In order to provision your device, vist the `$MIL_REPO/provisioning/android` and follow the instruction it its README.

(Also, send a message in slack on the software channel to let us know what andorid device you completed it on so that we can add it to the validated list.)

### Installing the MIL repo on your android device

* Clone the repo (see the contributing guide)

* Run the system and User install scripts (see the DEvelopment guide)

**NOTE: we are NOT running docker on the android device.**

* Compile the MIL Repo with `cm_arm` (this is different on the android device than it is on your personal computer because some of our source code has dependencies that are not supported on ARM CPU's, but are supported on amd64 CPU's)

### Install ROS Mobile On Your Device

* Go to the [ROS Mobile GitHub](https://github.com/ROS-Mobile/ROS-Mobile-Android)

* Follow the instructions there for installing the apk via adb

## Run the Whole System

* connect both the android device and your personal computer to the same Local Area Networ (LAN)

* Find the devices ip address with termux and the `ip a`

* ssh into the android device

* connect the usb camera to the android device

* make all of the data being passed between nodes on your android device availible to all devices on your LAN by running `ros_connect lan` on the android device (via the ssh session)

* attempt to run the whole sytem with `roslaunch mil_blind_spot_camera blind_spot_camera.launch`

* notice that it does not work due to the "usb_cam" node not being able to connect to `/dev/video0` This is due to how android workds, and the default on your android phone takes this place, and uses a differenct protocaul that the usb cameras.

To find the device that the usb camera will show up as:

* Unplug the usb camera from your android device.

* `ls /dev | grep video`

* Plug in the usb caemra to your android device

* `ls /dev | grep video`

There should be an additional video device listed in the second command. That is the one we want.

* Now that we know the video device that we want, we need to pass it in as an argument to the launch file argument. This launch file taked in an argument called `video_device`.

* Consult with the ros wiki to find out how to pass in a named argument.

* Now run the commaned with the correct vide device.

**NOTE: There may be some yellow warnings, these are generally not important and can be ignored for this project. Errors will show up as red and should NOT be ignored.**

### Verify that the system is working form your personal computer

* On your personal computer, open a terminal and export your `ROS_IP` and `ROS_MASTER_URI` environment variables to reflect the IP of your android device (in the correct formats).

* run `rviz` (in the same terminal where you export the environment variables).

* Click the `Add` button

* Click on the `By Topic` button

* Add all the available image topics.

* You should see some VERY low frame rate video coming from the camera from both of these topics.
(in my case it was about 2 frames per second)

* Use the command `rostopic list` to view all the available topics.

* Use the command `rostopic hz <insert topic name here>` to see the rate of images coming from the android device.

**NOTE: the robots in MIL have very fast network interfaces and CPU's, so when working with any of them, rivz runing on remote machines performs much better**

### Verify that the system is working form the android device

* Run the ROS Mobile App on the android device

* Create a new Configuration

* Under the "Master Tab", for the master URL, enter the android device's IP address.

* Press "Connect"

* Under Details, tap the plus button, tap the camera option, then select the `/usb_blind_spot_cam/image_raw`

* For X, enter 0

* For Y, enter 0

* For Width, enter 8

* For Height, enter 13

* Press the refresh symbol

* Select the "VIZ" Tab

* Notice that the image is horizontal, and kinda small.

* For this blind spot camera, we want it big.

* Because we are using ROS, we can write some code that will take this image, rotate it 90 degree clockwise, and publish the new image. This is what the `rotate_img` ROS node is.

* Under "DETAILS", select the topic name `/roated_img/image_raw`

* Press the Refresh button

* Go to the "VIZ" Tab.

* The image is now Vertical, and much bigger on the phone screen, which is exatly what we wanted for the blind spot caemra.

## Replaceing the launch file with your own

Launch files are possible the most important thing for all software engineers to understand in MIL.

A launch file is a file which can run ROS nodes and other launch files.

A launch file can accept arguments.

A launch file can pass arguments to anther launch file.

A launch file can pass parameters to ROS nodes.

* navigate to the launch directory in the `mil_blind_spot_camera` package on the android device with `roscd mil_blind_spot_camera/launch/`

* rename the current launch file with `mv start_usb_blind_spot_camera.launch start_usb_blind_spot_camera.launch.solution`

* rename the launch file skeleton with `mv start_usb_blind_spot_camera.launch.skeleton start_usb_blind_spot_camera.launch`

* open the launch file skeleton with nano or vim on the android device. Follow the comments and reference the ROS wiki on launch files to replicate the behavior of the old launch file

* Don't look at the solution file. That is cheating!

## Replace the image rotator with your own

`rotate_img` is a pyhton program that uses rospy to initialize a ROS node which subscribes to the image comming from the `usb_cam` node, rotates it 90 degrees clockwise, and publishes it.

`rotate_img` takes 2 private ROS parameters when it run which specify the topic names for the input image and output image.

* navigate to the soruce code directory in the `mil_blind_spot_camera` package on the android device with `roscd mil_blind_spot_camera/src/`

* rename the current launch file with `mv rotate_img rotate_img.solution`

* rename the launch file skeleton with `mv rotate_img.skeleton rotate_img`

* open the launch file skeleton with nano or vim on the android device. Follow the comments and reference the ROS wiki on rospy subscribers, publishers, and opencv to replicate the behavior of the old `rotate_img`

* Don't look at the solution file. That is cheating!

## Get your changes off of the android device and onto your personal computer

* On your personal computer, navigate to the mil repo with the `mil` command

* Use `git status` to see the status of your repository. Make sure to add and commit any changes you may change now.

* Use `git remote add android_device <androi_user>@<android_device_ip>:catkin_ws/src/mil/` to addthe android device as a git server.

* Use `git remote -v` to confirm that it was added as a remot repo.

* On the android device, checkout a new git branch in the mil repo with `git checkout -b my-exercises-branch`

* Add all the changes that you have so far. `mil && git add .`

* Commit this changes. `git commit -m "I made some changes"`

* On your personal computer, fetch all the changed branches from the android device. `git fetch android_device`

## Actually putting the setup in your car / vehicle

TODO

## Recording a bag of data from your vehicle

TODO

## Rectifying the camera to the ground for a birds-eye-view parking camera

TODO

