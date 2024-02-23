################################################################################
#  File name: tests.py
#  Author: Keith Khadar
#  Description: This file is used to store tests used by Preflight. If you want
#                add tests you would add them here.
################################################################################

#                  -----  Actuator Tests -----                  #
# Add tests here for actuators. These will be turned on and the user
# will confirm their operation. Also include any custom message
# imports here.

# Thruster Messages
from subjugator_msgs.msg import ThrusterCmd

actuatorsList = [
    (
        "/thrusters/thrust",
        [
            ThrusterCmd(name="FLH", thrust=10.0),
        ],
    ),
]

#                  -----  Hardware Tests -----                  #
# Add tests here for things that need to be physically inspected
hardware = [
    "Setup and connect to Network Box.",
    "Roll Tether and Connect it to network box. (DO NOT USE POE, Power-Over-Ethernet).",
    "Connect Sub to the tether. (NOT POE).",
    "Connect battery alarm. Power on Sub.",
    "SSH into Sub.",
    "Start tmux.",
    "Grease O-rings with Molykote 55 every time a pressure vessel is closed.",
    "Deploy sub. (Check for bubbles coming out of every pressure vessel, make sure buoyancy is correct)",
]


#                  -----  Software Tests -----                  #
# Add tests here for software systems like sensors whose operations
# need to be verified.

# -----    Nodes    -----#
nodes = ["/odom_estimator"]

# -----    Topics   -----#
topics = [
    "/camera/front/right/image_raw",
    "/camera/down/image_raw",
    "/camera/front/left/image_raw",
    "/dvl",
    "/depth",
    "/imu/data_raw",
    "/imu/mag",
]
