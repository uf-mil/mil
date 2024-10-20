#!/usr/bin/env python3

import contextlib
import json
import socket

import rospy
from geometry_msgs.msg import Point
from mil_passive_sonar.msg import ProcessedPing
from std_msgs.msg import Header


def main():
    # Define the server address and port
    HOST = "127.0.0.1"
    PORT = 2007  # The port used by the server

    # Create a socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        print(f"Connected to {HOST}:{PORT}")
        pub = rospy.Publisher("ping", ProcessedPing, queue_size=10)
        rospy.init_node("pingpublisher", anonymous=True)

        while not rospy.is_shutdown():
            # Receive data
            data = s.recv(1024)  # Buffer size is 1024 bytes
            if not data:
                break

            # Parse the JSON data
            try:
                json_data = json.loads(data.decode("utf-8"))
                # Create a ProcessedPing message
                ping_msg = ProcessedPing()

                # Populate the header
                ping_msg.header = Header()
                ping_msg.header.seq += 1  # Increment sequence number
                ping_msg.header.stamp = rospy.Time.now()
                ping_msg.header.frame_id = "ping_frame"  # Set your frame_id

                # Populate the position
                ping_msg.position = Point()
                ping_msg.position.x = json_data["origin_direction_body"][0]
                ping_msg.position.y = json_data["origin_direction_body"][1]
                ping_msg.position.z = json_data["origin_direction_body"][2]

                # Populate the frequency and amplitude
                ping_msg.freq = json_data["frequency_Hz"]
                ping_msg.amplitude = json_data[
                    "origin_distance_m"
                ]  # You can modify this as needed
                ping_msg.valid = True  # Or set to False based on your logic

                # Publish the message
                pub.publish(ping_msg)
                rospy.loginfo(f"Published: {ping_msg}")

            except json.JSONDecodeError as e:
                rospy.loginfo(f"JSON Decode Error: {e}")
            except KeyError as e:
                rospy.loginfo(f"Key Error: {e}")


if __name__ == "__main__":
    with contextlib.suppress(rospy.ROSInterruptException):
        main()
