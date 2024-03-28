################################################################################
#  File name: helper.py
#  Author: Keith Khadar
#  Description: This is used to store the helper functions for preflight
################################################################################
#                             -----  Imports -----                             #
# ----- Console  -----#
# ----- Async  -----#
import asyncio
import subprocess
import time

# ----- Misc  -----#
from contextlib import suppress

# ----- Preflight  -----#
import menus

# ----- ROS  -----#
import rospy
import rostopic
import tests
from axros import NodeHandle
from PyInquirer import prompt
from rich.console import Console
from rich.progress import Progress, track
from rich.table import Table

#                             -----  Variables -----                             #
report = []


#                             -----  Functions -----                             #
def clear_screen():
    # Clears the screen and prints the preflight header
    subprocess.run("clear", shell=True)
    Console().print(menus.title)


def init_report():
    # Function to initialize the report
    report.clear()


def init_ros():
    # Checks if ROS is running and initializes it.
    # Returns False if ROS is not running and TRUE if the ROS node was properly setup.

    # Check that ROS is running!
    try:
        rostopic._check_master()
    except Exception:
        Console().print("[bold] ROS not running! Please try again later[/]")
        prompt(menus.press_anykey)
        return False

    # Initialize the ROS node
    with suppress(Exception):
        rospy.init_node("preflight")
    return True


async def axros_check_nodes(nodes):
    # Asynchronously check all the nodes and then print/save the results

    # Setup AXROS
    nh = NodeHandle.from_argv("Preflight_nh", "", anonymous=True)

    # Using async.io check all the nodes
    answers = []
    async with nh:
        # Create all the function calls to check_node
        tasks = [check_node(node, answers, nh) for node in nodes]

        # Go through all the function calls and await them.
        # Using track to display a loading bar.
        for task in track(
            asyncio.as_completed(tasks),
            description="Checking Nodes...",
            total=len(tasks),
        ):
            await task

    # Clear the screen, print and save the response to the report
    print_results(answers, "Node Liveliness")


async def check_node(node, results, nh):
    # Using AXROS lookup a node and save the result in the results list

    try:
        results.append((node, bool(await nh.lookup_node(node))))
    except Exception:
        results.append((node, False))


async def axros_check_topics(topics):
    # Asynchronously check all the topics and then print/save the results

    # Setup AXROS
    nh = NodeHandle.from_argv("Preflight_nh", "", anonymous=True)

    # Using async.io check all the topics
    answers = []
    async with nh:
        # Create all the function calls to check topic
        tasks = [check_topic(topic, answers, nh) for topic in topics]

        # Go through all the function calls and await them.
        # Using track to display a loading bar.
        for task in track(
            asyncio.as_completed(tasks),
            description="Checking Topics...",
            total=len(tasks),
        ):
            await task

    # Clear the screen, print and save the response to the report
    print_results(answers, "Topic Liveliness")


async def check_topic(topic, results, nh):
    # Using AXROS subscribe to a topic and wait for a message

    # Get the topic class
    topicType, topicStr, _ = rostopic.get_topic_class(topic)

    # Create an AXROS subscriber
    sub = nh.subscribe(topicStr, topicType)

    async with sub:
        try:
            await asyncio.wait_for(sub.get_next_message(), tests.topic_timeout)
            results.append((topic, True))
        except Exception:
            results.append((topic, False))


def check_actuators(actuators):
    # Check all the actuators using check_actuator

    answers = []
    for actuator in actuators:
        check_actuator(actuator, answers)

    # Clear the screen, print and save the response to the report
    print_results(answers, "Actuator Tests")


def check_actuator(actuator, results):
    # Checks the actuator by publishing to a topic for a specified time

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


def generate_report():
    # Attempts to create and display the report

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


def print_results(systems, name):
    # This saves the result to the specified system and prints it
    clear_screen()
    result = create_result(systems, name)
    Console().print(result)


def create_result(systems, name):
    # This save the result into a RICH table

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
