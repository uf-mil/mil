import subprocess
import time

import menus
import rosnode
import rospy
import rostopic
from PyInquirer import prompt
from rich.console import Console
from rich.progress import Progress, track

# Custom message imports
from subjugator_msgs.msg import ThrusterCmd

node_timout = 5  # seconds
topic_timout = 5  # seconds
actuator_timout = 5  # seconds


hardwareChecklist = [
    {
        "type": "checkbox",
        "message": "Hardware Checklist:",
        "name": "HardwareTests: \nPlease check that all of the following are in working order. \nYou cannot continue until everything has been checked.",
        "choices": [
            {"name": "check thing 1"},
            {"name": "check thing 2"},
            {"name": "check thing 3"},
            {"name": "check thing 4"},
            {"name": "check thing 5"},
        ],
    },
]

topics = [
    # "/camera/front/right/image_raw",
    # "/camera/down/image_raw",
    # "/camera/front/left/image_raw",
    # "/dvl",
    # "/depth",
    # "/imu/data_raw",
    # "/imu/mag",
]

nodes = ["/odom_estimator"]

actuatorsList = [
    (
        "/thrusters/thrust",
        [
            ThrusterCmd(name="FLH", thrust=60.0),
            ThrusterCmd(name="FLV", thrust=60.0),
            ThrusterCmd(name="FRH", thrust=60.0),
            ThrusterCmd(name="FRV", thrust=60.0),
            ThrusterCmd(name="BLH", thrust=60.0),
            ThrusterCmd(name="BLV", thrust=60.0),
            ThrusterCmd(name="BRH", thrust=60.0),
            ThrusterCmd(name="BRV", thrust=60.0),
        ],
    ),
]


def main():
    # Clear Screen and Display Start menu
    subprocess.run("clear", shell=True)

    # Print Info about Preflight
    Console().print(menus.info_page)

    # Print start select menu
    option = prompt(menus.start_menu)
    mode = next(iter(option.values()))

    # Select the mode and run it
    if mode == "Run Preflight Full Test":
        fullTest()
    if mode == "Exit":
        subprocess.run("clear", shell=True)
        return

    # Return to this screen after running the selected mode
    main()


def fullTest():
    ### Complete the hardware tests ###

    while True:
        subprocess.run("clear", shell=True)
        answers = prompt(hardwareChecklist)

        # If everything has been checked off
        if len(next(iter(answers.values()))) == 5:
            break
        else:
            menu_ans = prompt(menus.incomplete_continue)
            if next(iter(menu_ans.values())) is True:
                # Continue even though everything has not been checked off
                break
            else:
                # Go back to main menu
                return

    ### Complete Software Tests ###

    subprocess.run("clear", shell=True)
    try:
        rostopic._check_master()
    except Exception:
        Console().print("[bold] ROS not running! Please try again later[/]")
        menu_ans = prompt(menus.press_anykey)
        return

    rospy.init_node("preflight")

    # Check Nodes
    answers = []
    for node in track(nodes, description="Checking Nodes..."):
        # Try and ping the nodes
        try:
            answers.append((node, rosnode.rosnode_ping(node, node_timout)))
        except Exception:
            answers.append((node, False))

    print_results(answers)
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Check Topics
    answers = []
    for topic in track(topics, description="Checking Topics..."):
        # Check for messages on the topics
        try:
            topicType, topicStr, _ = rostopic.get_topic_class(topic)  # get topic class
            rospy.wait_for_message(
                topicStr,
                topicType,
                topic_timout,
            )  # try to get a message from that topic
            answers.append({topic: True})
        except Exception:
            answers.append({topic: False})
    print_results(answers)

    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    ### Actuators Test ###
    subprocess.run("clear", shell=True)

    answers = []
    for actuator in actuatorsList:
        try:
            # Confirm that it is safe to run this actuator
            Console().print(menus.safety_check, actuator[0])
            menu_ans = prompt(menus.continue_question)
            if next(iter(menu_ans.values())) is False:
                # Go back to main menu
                return

            # Create a publisher
            topicType, topicStr, _ = rostopic.get_topic_class(actuator[0])
            pub = rospy.Publisher(topicStr, topicType, queue_size=10)

            # Publish to the topic for the specified timeout
            with Progress() as progress:
                t_start = time.time()
                t_end = t_start + actuator_timout
                t_prev = time.time()
                task = progress.add_task("Running", total=(t_end - t_start))
                while time.time() <= t_end:
                    pub.publish(actuator[1])
                    progress.update(task, advance=(time.time() - t_prev))
                    t_prev = time.time()
                progress.update(task, advance=t_end)

            # Ask if the actuator worked
            Console().print(menus.actuator_check)
            answers.append((actuator[0], next(iter(prompt(menus.yes_no).values()))))
        except Exception:
            answers.append((actuator[0], False))

    print_results(answers)
    menu_ans = prompt(menus.press_anykey)
    return


def print_results(systems):
    subprocess.run("clear", shell=True)
    for name, status in systems:
        if status:
            Console().print(f"{name}: [green]✔[/] Working")
        else:
            Console().print(f"{name}: [red]❌[/] Not Working")


if __name__ == "__main__":
    main()
