################################################################################
#  File name: main.py
#  Author: Keith Khadar
#  Description: This file is the entry point to preflight.
################################################################################
#                             -----  Imports -----                             #
# ----- Console  -----#
# ----- Async  -----#
import asyncio
import subprocess

# ----- Misc  -----#
from pathlib import Path

import helper

# ----- Preflight  -----#
import menus
import tests
from PyInquirer import prompt
from rich.console import Console
from rich.markdown import Markdown


#                             -----  Main Routine -----                           #
async def main():

    # Clear Screen and Display Start menu
    helper.clear_screen()

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
    except StopIteration:
        # Return if the user presses Ctrl-c
        return
    except Exception:
        pass

    # Return to this screen after running the selected mode
    await main()


#                            -----  Subroutines -----                          #


# ----- Modes  -----#
async def fullTest():

    helper.init_report()

    ### Complete the setup tests ###

    # Clear the screen and display the setup checklist
    helper.clear_screen()
    Console().print(menus.setup_desc)
    respond = prompt(menus.setupChecklist)

    # Filter the response and store checklist to the report
    answers = []
    for i in range(len(tests.setup)):
        if tests.setup[i] in next(iter(respond.values())):
            answers.append((tests.setup[i], True))
        else:
            answers.append((tests.setup[i], False))
    helper.create_result(answers, "Setup Checklist")

    # Check if the list is incomplete. If so prompt user for confirmation to continue
    if len(next(iter(respond.values()))) != len(tests.setup):
        menu_ans = prompt(menus.incomplete_continue)
        if next(iter(menu_ans.values())) is False:
            return

    ### Complete Software Tests ###
    if not helper.init_ros():
        return

    # Clear the screen
    helper.clear_screen()

    # Print Node Screen description
    Console().print(menus.node_desc)

    # Check Nodes
    await helper.axros_check_nodes(tests.nodes)

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Print Topic screen description
    helper.clear_screen()
    Console().print(menus.topic_desc)

    # Check Topics

    await helper.axros_check_topics(tests.topics)

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    ### Actuators Test ###
    # Print Actuators Screen description
    helper.clear_screen()
    Console().print(menus.node_desc)

    helper.check_actuators(tests.actuatorsList)

    prompt(menus.press_anykey_menu_return)
    return


def viewReport():
    # Clear the screen
    helper.clear_screen()

    # Generate the report
    helper.generate_report()
    return


def viewDocumentation():
    # Clear the screen
    helper.clear_screen()

    # Find path to README from current directory
    mod_path = Path(__file__).parent
    rel_path = "preflight.md"
    src_path = (mod_path / rel_path).resolve()
    # Print the documentation
    with open(src_path, "r+") as help_file:
        Console().print(Markdown(help_file.read()))

    prompt(menus.press_anykey_menu_return)
    return


async def specificTest():
    # Init the report
    helper.init_report()

    # Clear the screen and display the node checklist
    helper.clear_screen()

    helper.init_ros()

    # Clear the screen and display the node checklist
    helper.clear_screen()
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
    await helper.axros_check_nodes(nodes=nodes)

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Clear the screen and display the topic checklist
    helper.clear_screen()
    Console().print(menus.specific_desc)
    respond = prompt(menus.topicChecklist)

    # Filter the response and store checklist to the report
    topics = []
    for i in range(len(tests.topics)):
        if tests.topics[i] in next(iter(respond.values())):
            topics.append(tests.topics[i])

    # Print Topic screen description
    helper.clear_screen()
    Console().print(menus.topic_desc)

    # Check Topics
    await helper.axros_check_topics(topics=topics)

    # Prompt the user to continue to next test
    menu_ans = prompt(menus.continue_question)
    if next(iter(menu_ans.values())) is False:
        # Go back to main menu
        return

    # Clear the screen and display the actuator checklist
    helper.clear_screen()
    Console().print(menus.specific_desc)
    respond = prompt(menus.actuatorChecklist)

    # Filter the response and store checklist to the report
    actuators = []
    for i in range(len(tests.actuatorsList)):
        if tests.actuatorsList[i][0] in next(iter(respond.values())):
            actuators.append(tests.actuatorsList[i])

    # Print Actuators Screen description
    helper.clear_screen()
    Console().print(menus.node_desc)

    helper.check_actuators(actuators=actuators)

    prompt(menus.press_anykey_menu_return)
    return


if __name__ == "__main__":
    asyncio.run(main())
