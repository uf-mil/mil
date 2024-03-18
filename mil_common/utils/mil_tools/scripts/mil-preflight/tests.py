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

#                  ----- Timeouts  -----                 #
node_timeout = 5  # seconds
topic_timeout = 5  # seconds
actuator_timeout = 1.5  # seconds

actuatorsList = [
    (
        "FLH Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="FLH", thrust=10.0)]),
    ),
    (
        "FRH Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="FRH", thrust=10.0)]),
    ),
    (
        "BLH Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="BLH", thrust=10.0)]),
    ),
    (
        "BRH Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="BRH", thrust=10.0)]),
    ),
    (
        "FLV Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="FLV", thrust=10.0)]),
    ),
    (
        "FRV Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="FRV", thrust=10.0)]),
    ),
    (
        "BLV Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="BLV", thrust=10.0)]),
    ),
    (
        "BRV Thruster Test",
        ("/thrusters/thrust", [ThrusterCmd(name="BRV", thrust=10.0)]),
    ),
]

#                  -----  Setup Tests -----                  #
# Add tests here for things that need to be physically inspected or check before the sub is running
setup = [
    "Grease O-rings with Molykote 55 every time a pressure vessel is closed.",
    "Deploy sub. (Check for bubbles coming out of every pressure vessel, make sure buoyancy is correct)",
]


#                  -----  Software Tests -----                  #
# Add tests here for software systems like sensors whose operations
# need to be verified.

# -----    Nodes    -----#
nodes = [
    "/odom_estimator",
    "/odom_estimator",
    "/odom_estimator",
    "/odom_estimator",
    "/doest_exist",
]

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
