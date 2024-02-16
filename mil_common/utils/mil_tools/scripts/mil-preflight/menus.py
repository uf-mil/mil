################################################################################
#  File name: menus.py
#  Author: Keith Khadar
#  Description: This file is used to store all the menus used by preflight
################################################################################
# ----- Imports  -----#
import tests

# -----  Home Page  -----#
title = """
[bold green]Preflight Program - Autonomous Robot Verification[/bold green]
"""
info_page = """
Welcome to the Preflight Program, a tool inspired by the preflight checklists used by pilots before flying a plane. This program is designed to verify the functionality of all software and hardware systems on your autonomous robot. It ensures that everything is in working order, allowing you to safely deploy your robot with confidence.\n
[italic]Authors:[/italic]
Keith Khadar
Anthony Liao
Joshua Thomas
Maanas Kotha\n
"""


start_menu = [
    {
        "type": "list",
        "name": "mode selection",
        "message": "Menu",
        "choices": [
            "Run Preflight Full Test",
            "View Report",
            "Run Specific Test",
            "View Documentation",
            "Exit",
        ],
    },
]

# -----  Loading Screen  -----#
continue_question = [
    {
        "type": "confirm",
        "name": "continue",
        "message": "Continue?",
    },
]
press_anykey = [
    {
        "type": "confirm",
        "name": "continue",
        "message": "Press any key to continue.",
        "confirm": False,
    },
]

incomplete_continue = [
    {
        "type": "confirm",
        "name": "incomplete",
        "message": "This checklist is incomplete. Do you wish to continue?",
    },
]
yes_no = [
    {
        "type": "confirm",
        "name": "yes_no",
        "message": "Yes?",
    },
]


# -----  Software Screen  -----#
node_desc = """
Welcome to the [bold]ROS Node Liveliness Test screen[/]. This screen allows you to verify the liveliness of ROS nodes within the system. ROS nodes are essential components responsible for communication and control within the robot\'s architecture. Use this screen to ensure that all necessary nodes are running and responsive for proper system functionality.
"""
topic_desc = """
Welcome to the Topic Monitoring screen. This screen allows you to monitor the topics in the ROS system. In a ROS (Robot Operating System) environment, topics serve as channels through which nodes communicate by publishing and subscribing to messages. Monitoring topics enables you to verify that sensors are actively publishing their data.
"""

# -----  Hardware Screen  -----#
hardware_desc = """
Welcome to the [bold]Hardware Inspection Checklist[/] page of our preflight app! This checklist is designed to ensure that all critical hardware components of your robot are thoroughly inspected before each operation. By completing this checklist, you contribute to the safety and reliability of the robot during its mission. Please carefully examine each item listed below to verify its condition and functionality.
"""
hardwareChecklist = [
    {
        "type": "checkbox",
        "message": "Hardware Checklist:",
        "name": "HardwareTests: \nPlease check that all of the following are in working order.",
        "choices": [{"name": item} for item in tests.hardware],
    },
]

# -----  Actuators Screen  -----#
actuator_desc = """
Welcome to the [bold]Actuator Test Screen[/]. This screen allows you to test the functionality of various actuators on the robot. Actuators play a crucial role in the movement and operation of the robot. Use this screen with caution and follow the instructions carefully to avoid any accidents or damage to the robot.
\n[bold red]Caution:[/bold red] Actuators can be dangerous if mishandled. Please be careful when testing them, as they have the potential to cause harm or injury. For example, thrusters, when spinning, could chop off someone's finger. Always follow safety protocol and guidelines.
"""
safety_check = """
[bold][yellow]Ensure that all fingers are clear of the area![/yellow][/bold] Is it safe to operate the following actuator?
"""

actuator_check = """
Did the actuator work?
"""
