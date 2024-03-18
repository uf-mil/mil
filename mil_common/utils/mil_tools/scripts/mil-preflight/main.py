################################################################################
#  File name: main.py
#  Author: Keith Khadar
#  Description: This file is the entry point to preflight. Version 1.0
################################################################################
#                             -----  Imports -----                             #
# ----- Preflight  -----#
# ----- Console  -----#
# ----- Async  -----#
import asyncio
import subprocess
import time
from contextlib import suppress
from pathlib import Path

import menus

# ----- ROS  -----#
import rospy
import rostopic
import tests
from axros import NodeHandle
from PyInquirer import prompt
from rich.console import Console
from rich.markdown import Markdown
from rich.progress import Progress, track
from rich.table import Table

#                             -----  Variables -----                             #
# ----- Reports  -----#


report = []


#                             -----  Main Routine -----                           #
async def main():

    # Clear Screen and Display Start menu
    clear_screen()

    # Print Info about Preflight
    Console().print(menus.info_page)

    # Print start select menu
    option = prompt(menus.start_menu)
    try:
        mode = next(iter(option.values()))
        # Select the mode and run it
        if mode == "Run Preflight Full Test":
            await fullTest()
        if mode == "View Report":
            viewReport()
        if mode == "Run Specific Test":
            await specificTest()
        if mode == "View Documentation":
            viewDocumentation()
        if mode == "Exit":
            subprocess.run("clear", shell=True)
            return

    except Exception:
        pass

    # Return to this screen after running the selected mode
    await main()


#                            -----  Subroutines -----                          #

# ----- Modes  -----#


async def fullTest():
    # Clear the report
    report.clear()

    ### Complete the setup tests ###

    # Clear the screen and display the setup checklist
    clear_screen()
    Console().print(menus.setup_desc)
    respond = prompt(menus.setupChecklist)

    # Filter the response and store checklist to the report
    answers = []
    for i in range(len(tests.setup)):
        if tests.setup[i] in next(iter(respond.values())):
            answers.append((tests.setup[i], True))
        else:
            answers.append((tests.setup[i], False))
    createResult(answers, "Setup Checklist")

    # Check if the list is incomplete. If so prompt user for confirmation to continue
    if len(next(iter(respond.values()))) != len(tests.setup):
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
    with suppress(Exception):
        rospy.init_node("preflight")

    # Print Node Screen description
    Console().print(menus.node_desc)

    # Check Nodes

    # Setup AXROS
    nh = NodeHandle.from_argv("Preflight_nh", "", anonymous=True)
    async with nh:
        answers = []
        tasks = [check_node(node, answers, nh) for node in tests.nodes]
        for task in track(
            asyncio.as_completed(tasks),
            description="Checking Nodes...",
            total=len(tasks),
        ):
            await task

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

    # Setup AXROS
    nh = NodeHandle.from_argv("Preflight_nh", "", anonymous=True)
    async with nh:
        answers = []
        tasks = [check_topic(node, answers, nh) for node in tests.topics]
        for task in track(
            asyncio.as_completed(tasks),
            description="Checking Topics...",
            total=len(tasks),
        ):
            await task

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
        check_actuator(actuator, answers)

    # Clear the screen, print and save the response to the report
    print_results(answers, "Actuator Tests")
    prompt(menus.press_anykey_menu_return)
    return


async def specificTest():
    # Clear the report
    report.clear()

    # Clear the screen and display the node checklist
    clear_screen()

    # Check that ROS is running!
    try:
        rostopic._check_master()
    except Exception:
        Console().print("[bold] ROS not running! Please try again later[/]")
        menu_ans = prompt(menus.press_anykey)
        return

    # Initialize the ROS node
    with suppress(Exception):
        rospy.init_node("preflight")

    # Clear the screen and display the node checklist
    clear_screen()
    Console().print(menus.specific_desc)
    respond = prompt(menus.nodeChecklist)

    # Filter the response and store checklist to the report
    nodes = []
    for i in range(len(tests.nodes)):
        if tests.nodes[i] in next(iter(respond.values())):
            nodes.append(tests.nodes[i])

    # Print Node Screen description
    Console().print(menus.node_desc)

    # Check Nodes
    nh = NodeHandle.from_argv("Preflight_nh", "", anonymous=True)
    async with nh:
        answers = []
        tasks = [check_node(node, answers, nh) for node in nodes]
        for task in track(
            asyncio.as_completed(tasks),
            description="Checking Nodes...",
            total=len(tasks),
        ):
            await task

    # Clear the screen, print and save the response to the report
    print_results(answers, "Node Liveliness")

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Clear the screen and display the topic checklist
    clear_screen()
    Console().print(menus.specific_desc)
    respond = prompt(menus.topicChecklist)

    # Filter the response and store checklist to the report
    topics = []
    for i in range(len(tests.topics)):
        if tests.topics[i] in next(iter(respond.values())):
            topics.append(tests.topics[i])

    # Print Topic screen description
    clear_screen()
    Console().print(menus.topic_desc)

    # Check Topics
    nh = NodeHandle.from_argv("Preflight_nh", "", anonymous=True)
    async with nh:
        answers = []
        tasks = [check_topic(node, answers, nh) for node in topics]
        for task in track(
            asyncio.as_completed(tasks),
            description="Checking Topics...",
            total=len(tasks),
        ):
            await task

    # Clear the screen, print and save the response to the report
    print_results(answers, "Topic Liveliness")

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Clear the screen and display the actuator checklist
    clear_screen()
    Console().print(menus.specific_desc)
    respond = prompt(menus.actuatorChecklist)

    # Filter the response and store checklist to the report
    actuators = []
    for i in range(len(tests.actuatorsList)):
        if tests.actuatorsList[i][0] in next(iter(respond.values())):
            actuators.append(tests.actuatorsList[i])

    # Print Actuators Screen description
    subprocess.run("clear", shell=True)
    Console().print(menus.node_desc)

    answers = []
    for actuator in actuators:
        check_actuator(actuator, answers)

    # Clear the screen, print and save the response to the report
    print_results(answers, "Actuator Tests")
    prompt(menus.press_anykey_menu_return)
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
    prompt(menus.press_anykey_menu_return)
    return


def viewDocumentation():
    # Clear the screen
    clear_screen()

    # Find path to README from current directory
    mod_path = Path(__file__).parent
    rel_path = "README.md"
    src_path = (mod_path / rel_path).resolve()
    # Print the documentation
    with open(src_path, "r+") as help_file:
        Console().print(Markdown(help_file.read()))

    prompt(menus.press_anykey_menu_return)
    return


# ----- Helper  -----#


def createResult(systems, name):
    # Generates a table to hold information about each system
    result = Table(title=f"[bold]{name}[/]")
    result.add_column("System Name", justify="center", style="cyan", no_wrap=True)
    result.add_column("Status", justify="center", style="magenta", no_wrap=True)

    # Populates the table
    for system, status in systems:
        status_text = "[green]✔ Working[/]" if status else "[red]❌ Not Working[/]"
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


async def check_node(node, results, nh):
    try:
        results.append((node, bool(await nh.lookup_node(node))))
    except Exception:
        results.append((node, False))


async def check_topic(topic, results, nh):
    topicType, topicStr, _ = rostopic.get_topic_class(topic)  # get topic class
    sub = nh.subscribe(topicStr, topicType)

    async with sub:
        try:
            await asyncio.wait_for(sub.get_next_message(), tests.topic_timeout)
            results.append((topic, True))
        except Exception:
            results.append((topic, False))


def check_actuator(actuator, results):
    try:
        # Confirm that it is safe to run this actuator
        Console().print(menus.safety_check, actuator[0])
        menu_ans = prompt(menus.continue_question)
        if next(iter(menu_ans.values())) is False:
            # Go back to main menu
            return

        # Create a publisher
        topicType, topicStr, _ = rostopic.get_topic_class(actuator[1][0])
        pub = rospy.Publisher(topicStr, topicType, queue_size=10)

        # Publish to the topic for the specified timeout
        with Progress() as progress:
            t_start = time.time()
            t_end = t_start + tests.actuator_timeout
            t_prev = time.time()
            task = progress.add_task("Running", total=(t_end - t_start))
            while time.time() <= t_end:
                pub.publish(actuator[1][1])
                progress.update(task, advance=(time.time() - t_prev))
                t_prev = time.time()
            progress.update(task, advance=t_end)

        # Ask if the actuator worked
        Console().print(menus.actuator_check)
        results.append((actuator[0], next(iter(prompt(menus.yes_no).values()))))
    except Exception:
        Console().print(menus.actuator_failed)
        results.append((actuator[0], False))


if __name__ == "__main__":
    asyncio.run(main())
