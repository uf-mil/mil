import subprocess

import rosnode
import rospy
import rostopic
import typer
from PyInquirer import prompt
from rich.console import Console
from rich.progress import track

app = typer.Typer()


def display_start_menu():
    console = Console()

    # Title
    console.print(
        "[bold green]Preflight Program - Autonomous Robot Verification[/bold green]",
    )

    # Description
    console.print(
        "Welcome to the Preflight Program, a tool inspired by the preflight checklists used by pilots before "
        "flying a plane. This program is designed to verify the functionality of all software and hardware "
        "systems on your autonomous robot. It ensures that everything is in working order, allowing you to "
        "safely deploy your robot with confidence.\n",
    )

    # Authors section
    console.print("\n[italic]Authors:[/italic]")
    console.print("Keith Khadar")
    console.print("Anthony Liao")
    console.print("Joshua Thomas\n")

    # Menu options
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
    option = prompt(start_menu)
    return next(iter(option.values()))


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
    "/camera/front/right/image_raw",
    "/camera/down/image_raw",
    "/camera/front/left/image_raw",
    "/dvl",
    "/depth",
    "/imu/data_raw",
    "/imu/mag",
]

nodes = ["/odom_estimator"]

actuatorsList = {"/thrusters/thrust": ["FLH", 25.0]}


@app.command("Start")
def main():
    # Display Modes/Options
    subprocess.run("clear", shell=True)
    mode = display_start_menu()

    if mode == "Run Preflight Full Test":
        hardware()
        software()
        actuators()
    if mode == "Exit":
        subprocess.run("clear", shell=True)
        return
    main()


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
    print("test")
    answers = []
    try:
        prompt(
            [
                {
                    "type": "confirm",
                    "name": "runActuator",
                    "message": "Are your sure you want to run "
                    + actuatorsList.keys[0]
                    + "? BE CAREFUL make sure everyone's fingures are secured.",
                },
            ],
        )
        topicType, topicStr, _ = rostopic.get_topic_class(
            actuatorsList.keys[0],
        )  # get topic class
        pub = rospy.Publisher(topicStr, topicType, queue_size=10)
        rostopic.publish_message(pub, topicType, actuatorsList.values[0])

        answers.append(
            prompt(
                [
                    {
                        "type": "confirm",
                        "name": "worked?",
                        "message": "Did "
                        + actuatorsList.keys[0]
                        + " work as expected?",
                    },
                ],
            ),
        )
    except Exception as e:
        print(e)
        answers.append(False)

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


if __name__ == "__main__":
    app()
