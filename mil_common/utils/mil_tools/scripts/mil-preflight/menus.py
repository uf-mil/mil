################################################################################
#  File name: menus.py
#  Author: Keith Khadar
#  Description: This file is used to store all the menus used by prefligh
################################################################################


# -----  Home Page  -----#
info_page = """
[bold green]Preflight Program - Autonomous Robot Verification[/bold green]
Welcome to the Preflight Program, a tool inspired by the preflight checklists used by pilots before flying a plane. This program is designed to verify the functionality of all software and hardware systems on your autonomous robot. It ensures that everything is in working order, allowing you to safely deploy your robot with confidence.\n
[italic]Authors:[/italic]
Keith Khadar
Anthony Liao
Joshua Thomas\n
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

# -----  Actuators Screen  -----#
safety_check = """
MAKE SURE THAT EVERYONE'S FINGERS ARE CLEAR!! Is it safe to run the following actuator?
"""

actuator_check = """
Did the actuator work?
"""
