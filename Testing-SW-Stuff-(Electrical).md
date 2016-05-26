**Testing SW Stuff (Electrical)**

Move thruster:

roslaunch sub8_launch sub8.launch

rostopic pub /thrusters/thrust sub8_msgs/Thrust Thrust.msg **tab** **tab**

OR

python thruster.py -c /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A403IMC9-if00-port0 -i 11 -m 0 0.1