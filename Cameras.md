# Cameras
Sub8's visual stack will contain three cameras. They are the following: 

* __Stereo System:__ Two Point Grey Chameleon3s (CM3-13S2C) 
* __Downward System:__ One Point Grey Blackfly (U3-13S2C) 

# At-a-glance Info
Bits of information you might need if you're poking around with the cameras

* __Camera GUIDs:__ 
    * __Left Stereo:__ 00b09d0100ea51c4
    * __Right Stereo:__ 00b09d0100ea51be
    * __Downward:__ 00b09d0100eabf6c

* __Bus Transfer Speed:__ We're running the cameras in format7_mode1 (644x482 pixels) with raw8 for color information. That gives us the following breakdown: 644 * 482 * 30 (fps) * 8 (raw_8) = ~75 MB/s. For a total of three cameras, we will be using a total of 225 MB/s (Can be accommodated on a single USB3 bus).
 
* __Justifications for Camera Options:__ The vision tasks for the AUVSI RoboSub competition do not require a large amount of resolution. All of the cameras that were selected for this vehicle contain the same sensor (Sony ICX445 CCD) with a 1288x964 resolution. Computationally it is much easier to work with lower resolution images, especially if we intend to do stereo correspondence for SLAM/Visual odometry. If it is needed we later decide to run the cameras at a higher resolution. 

* __Useful Camera Registers:__ 0x604 for video_mode and 0x608 for video_format.  

* __Notes:__

    + All of the cameras contain an internal FPGA that handles debayering the images for color images. Typically image_raw in ROS would have to be [debayered using image_proc](http://wiki.ros.org/image_proc#image_proc.2BAC8-diamondback.image_proc.2BAC8-debayer), but in our case we can use image_raw without any problems. As a consequence we cannot decide how were introduce color information into the image (Billinear, EdgeAware, etc.). 
By running the cameras in format7mode0 we can do this, but we cannot downsample the image. 

    + Running the cameras in format7mode4 allows to run at ~50fps but only in mono. The image is incredibly sharp, but alas, we need the color information. 

# Dependencies/Setup

* libusb 1.0
* libdc1394 (Modified)
* libdc1394-utils
* coriander 2.0.2
* flycapture 2.6.3.4 (Used for properly setting the udev/group policies)
* camera1394 (ROS package)

## libusb 1.0
In Ubuntu run: `sudo apt-get install libusb-1.0-0 libusb-1.0-0-dev`.

## libdc1394
1. Download the latest version of [libdc1394](http://sourceforge.net/projects/libdc1394/)
2. Untar the file, and navigate to libdc1394-2.2.X/dc1394/usb. Open control.c in your preferred text editor.
3. Add `{ 0x1e10, 0x3300 }, // Point Grey BlackFly Color` to the end of `usb_products[] = { ...` on line 69.
4. Return to libdc1394-2.2.X/ and run `./configure`, `make`, and `sudo make install`.

This will build and install a properly patched version of the lib1394 library to your system. 

## libdc1394-utils
In Ubuntu run `sudo apt-get install libdc1394-utils`. Do this for the sake of your sanity. 

## Coriander
In Ubuntu run: `sudo apt-get install coriander`.
This will program will be used for setting the frame rate on the cameras.

## FlyCapture
1. Download [FlyCapture 2.6.3.4](https://www.ptgrey.com/support/downloads) __Account on website is required__.
2. Unpack the download. 
3. Run `./install_flycapture.sh`

## Camera1394
1. Clone [camera1394](https://github.com/ros-drivers/camera1394) repo into the src directory of your ROS workspace. 
2. `catkin_make`

Once all of this has been done, go ahead and reboot your computer. 

# Testing/Running

The first time you run the camera system you will have to run coriander and set the frame rate on each of the cameras to ~30. This is accomplished by:

1. Under the "Camera" tab select the camera whose settings you want to modify
2. Select the Controls tab, and navigate to the bottom of the page
3. Select the "Abs" option, and set it to 30
4. Repeat for the other cameras onboard
5. Reboot system

This process should only happen __once__, that is, when the camera system is first installed. 
After this, we can simply run `roslaunch camera1394 stereo.launch` to launch the camera system. 

# Hiccups

IIDC cameras are __EXTREMELY__ finicky. Changing certain settings will cause them to go blank, and nothing will seem to revive the cameras except reconnecting them. If this happens you will have to reset the firewire bus. This is done by running `dc1394_reset_bus`. Keep this command handy, you'll be running it plenty if you decide to poke around with the cameras. 

After all of this you might be wondering why we're not using the [ros_pointgrey_driver](https://github.com/ros-drivers/pointgrey_camera_driver) instead of doing all of this stuff. As of Dec. 2015, the driver is not usable for our purposes. Running more than one flycapture instance was __INCREDIBLY__ inconsistent and finicky. 

Additionally, killing specific cameras via software has not been implemented. The utility provided kills a random camera, and when run multiple times it will always kill that camera. A reset utility still has to be made.

# Notes on the downward camera

The downward-facing camera is using a 1.55mm lens. This gives us a FOV of approximately 140°, which means that we should be treating this as an omnidirectional camera. Calibration will have to carried out using OCamCalib (???) or some other yet-to-be-determined package. Updates will be posted shortly.  

# TODO
- [ ] Add camera reset utility (HIGH PRIORITY)
- [ ] Add lens selection & calibration parameters 
- [ ] Automate camera installation process
- [ ] Set frame rate without the use of coriander