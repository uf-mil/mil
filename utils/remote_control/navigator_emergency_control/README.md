This package contains nodes for interfacing with the emergency controller receiver board, its firmware, and bootloader.

The 'nodes' folder contains a python node for listening to the /joy_emergency topic and publishes to /wrench/cmd when the emergency controller is selected as the active controller. 

The 'navigator_emergency_control' folder contains the firmware and bootloader for the receiver board, and a c++ driver node which publishes incoming joy data from the uP to /joy_emergency. The code for connecting to the stm32f3 uP through usb and corresponding bootloader is hijacked from Forrest Voight's stm32f3discovery_imu_driver package in hardware-common.

The uP firmware interprets serial messages from the xbee as Joy data. If the controller stops sending messages, the c++ node times out and zeros the joy data. If the nodes are run with the hardware not connected, the launcher simply fails to open the port and continues operation. If the hardware disconnects mid operation, or the c++ node crashes for whatever reason, the python node zeros the wrench data after 2 seconds.

To test the functionality of the emergency controller with the simulator, simply connect the receiver board by usb and run:

    roslaunch navigator_launch simulation.launch

Hold the "start" button for a few seconds to set the controller to active. Then, check the output of topics /joy_emergency, /wrench/emergency, and /wrench/cmd.
