#!/usr/bin/env python3

import argparse
import threading

import rospy
from roboteq_msgs.msg import Command


def publish_thruster_command(thruster, rate):
    topic = f"/{thruster}_motor/cmd"
    command = Command(mode=0, setpoint=float(rate))
    pub = rospy.Publisher(topic, Command, queue_size=10)
    rate_obj = rospy.Rate(rate)
    rospy.loginfo(f"Starting to publish to {topic} with setpoint: {rate}")
    while not rospy.is_shutdown():
        pub.publish(command)
        rate_obj.sleep()


def main():
    parser = argparse.ArgumentParser(
        description="Spin thrusters for the autonomous boat",
    )

    parser.add_argument("--all", action="store_true", help="Spin all thrusters")
    parser.add_argument("--rate", type=float, default=0, help="Set thruster spin rate")
    parser.add_argument("--slow", action="store_true", help="Set slow rate (50)")
    parser.add_argument("--medium", action="store_true", help="Set medium rate (100)")
    parser.add_argument("--fast", action="store_true", help="Set fast rate (200)")
    parser.add_argument(
        "thrusters",
        nargs="*",
        help="List of thruster names to spin (FR, FL, BR, BL)",
    )

    args = parser.parse_args()

    # Define default rates based on arguments
    if args.slow:
        rate = 50
    elif args.medium:
        rate = 100
    elif args.fast:
        rate = 200
    elif args.rate > 0:
        rate = args.rate
    else:
        rospy.logerr("No rate specified! Use --rate, --slow, --medium, or --fast.")
        return

    # Initialize ROS node
    rospy.init_node("thruster_spin_control", anonymous=True)

    # Determine thrusters to spin
    thrusters = []
    if args.all:
        thrusters = ["FR", "FL", "BR", "BL"]
    elif args.thrusters:
        thrusters = args.thrusters
    else:
        rospy.logerr(
            "No thrusters specified! Use --all or list specific thrusters (FR, FL, BR, BL).",
        )
        return

    # Start spinning each thruster in a separate thread
    threads = []
    for thruster in thrusters:
        rospy.loginfo(f"Starting {thruster} at rate {rate}")
        thread = threading.Thread(
            target=publish_thruster_command,
            args=(thruster, rate),
        )
        thread.start()
        threads.append(thread)

    # Wait for all threads to complete
    for thread in threads:
        thread.join()


if __name__ == "__main__":
    main()
