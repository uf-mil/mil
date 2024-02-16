################################################################################
#  File name: main.py
#  Author: Keith Khadar
#  Description: This file is the entry point to preflight.
################################################################################
#                             -----  Imports -----                             #
# ----- Preflight  -----#
# ----- Console  -----#
import subprocess
import time

import menus

# ----- ROS  -----#
import rosnode
import rospy
import rostopic
import tests
from PyInquirer import prompt
from rich.console import Console
from rich.progress import Progress, track
from rich.table import Table

#                             -----  Variables -----                             #
# ----- Timeouts  -----#
node_timout = 5  # seconds
topic_timout = 5  # seconds
actuator_timout = 5  # seconds

# ----- Reports  -----#
report = []


#                             -----  Main Routine -----                           #


def main():
    # Clear Screen and Display Start menu
    clear_screen()

    # Print Info about Preflight
    Console().print(menus.info_page)

    # Print start select menu
    option = prompt(menus.start_menu)
    mode = next(iter(option.values()))

    # Select the mode and run it
    if mode == "Run Preflight Full Test":
        fullTest()
    if mode == "View Report":
        viewReport()
    if mode == "Exit":
        subprocess.run("clear", shell=True)
        return

    # Return to this screen after running the selected mode
    main()


#                            -----  Subroutines -----                          #

# ----- Modes  -----#


def fullTest():
    ### Complete the hardware tests ###

    # Clear the screen and display the hardware checklist
    clear_screen()
    Console().print(menus.hardware_desc)
    respond = prompt(menus.hardwareChecklist)

    # Filter the response and store checklist to the report
    answers = []
    for i in range(len(tests.hardware)):
        if tests.hardware[i] in next(iter(respond.values())):
            answers.append((tests.hardware[i], True))
        else:
            answers.append((tests.hardware[i], False))
    createResult(answers, "Hardware Checklist")

    # Check if the list is incomplete. If so prompt user for comfirmation to continue
    if len(next(iter(respond.values()))) != len(tests.hardware):
        menu_ans = prompt(menus.incomplete_continue)
        if next(iter(menu_ans.values())) is False:
            return

    ### Complete Software Tests ###

    # Clear the screen
    clear_screen()

    # Check that ROS is running!
    try:
        rostopic._check_master()
    except Exception:
        Console().print("[bold] ROS not running! Please try again later[/]")
        menu_ans = prompt(menus.press_anykey)
        return

    # Initialize the ROS node
    rospy.init_node("preflight")

    # Print Node Screen description
    Console().print(menus.node_desc)

    # Check Nodes
    answers = []
    for node in track(tests.nodes, description="Checking Nodes..."):
        # Try and ping the nodes
        try:
            answers.append((node, rosnode.rosnode_ping(node, node_timout)))
        except Exception:
            answers.append((node, False))

    # Clear the screen, print and save the response to the report
    print_results(answers, "Node Liveliness")

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Print Topic screen description
    clear_screen()
    Console().print(menus.topic_desc)

    # Check Topics
    answers = []
    for topic in track(tests.topics, description="Checking Topics..."):
        # Check for messages on the topics
        try:
            topicType, topicStr, _ = rostopic.get_topic_class(topic)  # get topic class
            rospy.wait_for_message(
                topicStr,
                topicType,
                topic_timout,
            )  # try to get a message from that topic
            answers.append((topic, True))
        except Exception:
            answers.append((topic, False))

    # Clear the screen, print and save the response to the report
    print_results(answers, "Topic Liveliness")

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    ### Actuators Test ###
    # Print Actuators Screen description
    subprocess.run("clear", shell=True)
    Console().print(menus.node_desc)

    answers = []
    for actuator in tests.actuatorsList:
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

    # Clear the screen, print and save the response to the report
    print_results(answers, "Actuator Tests")
    prompt(menus.press_anykey)
    return


def viewReport():
    # Clear the screen
    clear_screen()

    # Check that there is a report
    if len(report) == 0:
        Console().print(
            "[bold]No report![/].\nPlease generate a report by running a full test.",
        )
        prompt(menus.press_anykey)
        return
    # Generate the report
    for result in report:
        Console().print(result)
    prompt(menus.press_anykey)
    return


# ----- Helper  -----#


def createResult(systems, name):
    # Generates a table to hold information about each system
    result = Table(title=f"[bold]{name}[/]")
    result.add_column("System Name", justify="center", style="cyan", no_wrap=True)
    result.add_column("Status", justify="center", style="magenta", no_wrap=True)

    # Populates the table
    for system, status in systems:
        status_text = "[green]✔[/] Working" if status else "[red]❌[/] Not Working"
        result.add_row(system, status_text)
    report.append(result)

    return result


def print_results(systems, name):
    clear_screen()
    result = createResult(systems, name)
    Console().print(result)


def clear_screen():
    subprocess.run("clear", shell=True)
    Console().print(menus.title)


if __name__ == "__main__":
    main()
