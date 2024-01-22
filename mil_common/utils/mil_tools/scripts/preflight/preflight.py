import subprocess

import rosnode
import rospy
import rostopic
import typer
from PyInquirer import prompt

app = typer.Typer()

hardwareChecklist = [
    {
        "type": "checkbox",
        "message": "Hardware Checklist:",
        "name": "HardwareTests",
        "choices": [
            {"name": "check thing 1"},
            {"name": "check thing 2"},
            {"name": "check thing 3"},
            {"name": "check thing 4"},
            {"name": "check thing 5"},
        ],
    },
]
mechanicalChecklist = [
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

run = ["/thrusters/thrust"]


@app.command("Start")
def main():
    subprocess.run("clear", shell=True)
    answers = prompt(hardwareChecklist)
    while len(answers["HardwareTests"]) != 5:
        subprocess.run("clear", shell=True)
        answers = prompt(hardwareChecklist)
        print(answers)


@app.command("Topic")
def topic():
    rospy.init_node("preflight")
    answers = []
    for topic in topics:
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


@app.command("Node")
def node():
    rospy.init_node("preflight")
    answers = []
    for node in nodes:
        try:
            answers.append({node: rosnode.rosnode_ping(node, 5)})
        except Exception:
            answers.append({node: False})
    print(answers)


@app.command("Run")
def Run():
    rospy.init_node("preflight")


if __name__ == "__main__":
    app()
