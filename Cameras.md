# Cameras

As of right now we will be using two Point Grey Blackfly U3-23S6Cs. They will serve as the forward facing cameras. Plans are in place for a third, downward facing camera.

# Dependencies/Setup

* libusb 1.0
* libdc1394 (Modified)
* coriander 2.0.2
* flycapture 2.6.4 (Used for properly setting the udev/group policies)
* camera1394 (ROS package)

## libusb 1.0
In Ubuntu run: 'sudo apt-get install libusb-1.0-0 libusb-1.0-1-dev

## libdc1394
1. Download the latest version of [libdc1394] (http://sourceforge.net/projects/libdc1394/)
2. Untar the file, and navigate to libdc1394-2.2.X/dc1394/usb. Open control.c in your preferred text editor.
3. Add '{ 0x1e10, 0x3300 }, // Point Grey BlackFly Color' to the end of 'usb_products[] = { ...' on line 69.
4. Return to libdc1394-2.2.X/ and run './configure', 'make', and 'sudo make install'

This will install a properly patched version of the lib1394 library to your system. 

## Coriander
In Ubuntu run: 'sudo apt-get install coriander'
This will program will mainly be used for setting the frame rate on the cameras.

## FlyCapture


# Running Everything

# Hiccups

# Todo
1. Automate this process
2. Set frame rate without the use of coriander
