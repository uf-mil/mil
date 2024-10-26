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
    PORT = 2007

    # Create a socket
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        pub = rospy.Publisher("hydrophones/solved", ProcessedPing, queue_size=10)
        s.connect((HOST, PORT))
        rospy.loginfo(
            f"\nConnected to {HOST}:{PORT}\nPublishing to /hydrophones/solved",
        )

        # Need to ignore the first 2 JSON Parse Errors (nothing wrong)
        parse_error_count = 0

        while not rospy.is_shutdown():
            # Receive data
            data = s.recv(1024)
            if not data:
                break

            # Parse the JSON data
            try:
                json_data = json.loads(data.decode("utf-8"))
                ping_msg = ProcessedPing()

                # Populate the header
                ping_msg.header = Header()
                ping_msg.header.seq += 1
                ping_msg.header.stamp = rospy.Time.now()
                ping_msg.header.frame_id = "hydrophones"

                # Populate the position
                ping_msg.position = Point()
                ping_msg.position.x = json_data["origin_direction_body"][0]
                ping_msg.position.y = json_data["origin_direction_body"][1]
                ping_msg.position.z = json_data["origin_direction_body"][2]

                # Populate the frequency and amplitude
                ping_msg.freq = json_data["frequency_Hz"]
                ping_msg.amplitude = json_data["origin_distance_m"]
                ping_msg.valid = True

                # Publish the message
                pub.publish(ping_msg)
            except json.JSONDecodeError as e:
                parse_error_count += 1
                # ignore first two (normal behavior)
                if parse_error_count > 2:
                    rospy.logerr(f"JSONDecodeError: {e}")
            except KeyError as e:
                rospy.logerr(f"Key Error: {e}")


if __name__ == "__main__":
    with contextlib.suppress(rospy.ROSInterruptException):
        rospy.init_node("pingpublisher", anonymous=True)
        main()
