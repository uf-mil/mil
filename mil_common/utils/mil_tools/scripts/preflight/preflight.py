import subprocess

import preflight_menus
import rosnode
import rospy
import rostopic
import typer
from PyInquirer import prompt
from rich.progress import track

app = typer.Typer()

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
actuatorChecklist = [
    {
        "type": "list",
        "message": "Select which system you want to run. BE CAREFUL make sure everyone's fingures are secured.",
        "name": "Mechanical Systems Check:",
        "choices": ["thrusters", "Gripper"],
    },
]

topics = [
    "/camera/front/right/image_raw",
    "/camera/down/image_raw",
    "/camera/front/left/image_raw",
    "/dvl",
    "/depth",
    "/imu/data_raw",
    "/imu/mag",
]

nodes = ["/odom_estimator"]

actuatorsList = ["/thrusters/thrust"]


@app.command("Start")
def main():
    # Display Modes/Options
    subprocess.run("clear", shell=True)
    mode = preflight_menus.display_start_menu()

    if mode == "Run Preflight Full Test":
        hardware()
        software()
        actuators()

    # Complete the actuator tests
    subprocess.run("clear", shell=True)


def hardware():
    # Complete the hardware tests
    subprocess.run("clear", shell=True)
    answers = prompt(hardwareChecklist)
    while len(next(iter(answers.values()))) != 5:
        subprocess.run("clear", shell=True)
        answers = prompt(hardwareChecklist)


def software():
    # Complete the software tests
    subprocess.run("clear", shell=True)
    rospy.init_node("preflight")

    # Check Nodes
    answers = []
    for node in track(nodes, description="Checking Nodes..."):
        try:
            answers.append({node: rosnode.rosnode_ping(node, 5)})
        except Exception:
            answers.append({node: False})
    print(answers)

    # Check Topics
    answers = []
    for topic in track(topics, description="Checking Topics..."):
        try:
            topicType, topicStr, _ = rostopic.get_topic_class(topic)  # get topic class
            rospy.wait_for_message(
                topicStr,
                topicType,
                5,
            )  # try to get a message from that topic
            answers.append({topic: True})
        except Exception:
            answers.append({topic: False})
    print(answers)

    print(
        prompt(
            [
                {
                    "type": "confirm",
                    "name": "continue",
                    "message": "Continue?",
                },
            ],
        ),
    )


def actuators():
    subprocess.run("clear", shell=True)


if __name__ == "__main__":
    app()
