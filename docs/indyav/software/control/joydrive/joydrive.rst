IndyAV Joydrive Utility
=======================


How to Use
----------

Setting up an Xbox 360 controller
*********************************
``roslaunch indyav_launch gazebo.launch``

Plug in the controller and check to see if it is recognized.
``ls /dev/input/``
This outputs a list of input devices in the format shown below::

    by-id    event0  event2  event4  event6  event8  mouse0  mouse2  uinput
    by-path  event1  event3  event5  event7  js0     mice    mouse1

Joysticks will be listed as jsX.

Running the utility
*******************
Launch the Joydrive.
``roslaunch indyav_launch joydrive.launch``
By default, js0 is used by the joydrive utility. To change the input device, pass the name of the input device using the "js" argument.
``roslaunch indyav_launch joydrive.launch js:=js1``

In a new panel, launch Gazebo for IndyAV.
``roslaunch indyav_launch gazebo.launch``

In a new panel, launch RVIZ for IndyAV.
``indyviz``

In a new panel, launch the Gazebo Client.
``gazebogui``

Using the utility
*****************
Moving the left joystick left and right steers the car.

Depressing the right trigger accelerates the car. Releasing the trigger brakes the car.

Optionally, one can view the values for the wheel velocity and steering angle by running the following in new panels:
``rostopic echo /throttle``
``rostopic echo /steering``
