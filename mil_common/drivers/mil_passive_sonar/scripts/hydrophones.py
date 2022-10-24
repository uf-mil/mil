#!/usr/bin/env python3
import math

import numpy
import rosbag
import rospy
from geometry_msgs.msg import Point, Vector3Stamped
from mil_passive_sonar import algorithms, util
from mil_passive_sonar.msg import Debug, Ping, ProcessedPing
from mil_tools import numpy_to_vector3


def process_ping(ping: Ping):
    """
    Processes a ping.
    """
    ping.header.frame_id = "hydrophones"
    samples = util.ping_to_samples(ping)
    sample_rate = ping.sample_rate

    r = algorithms.run(samples, sample_rate, v_sound, dist_h, dist_h4)

    if len(r["pos"]) > 0:
        heading = math.atan2(r["pos"][1], r["pos"][0])
        declination = math.atan2(-r["pos"][2], numpy.linalg.norm(r["pos"][0:2]))
    else:
        heading = 0
        declination = 0

    if len(r["errors"]) > 0:
        rospy.logwarn("Errors processing ping: " + ", ".join(r["errors"]))

    valid = len(r["pos"]) > 0 and len(r["errors"]) == 0
    if valid:
        pos2 = r["pos"] / numpy.linalg.norm(r["pos"])
        print("result angle:", numpy.rad2deg(numpy.arctan2(pos2[1], pos2[0])) % 360)

        pub.publish(
            header=ping.header,
            position=Point(*r["pos"].tolist()),
            freq=r["freq"],
            amplitude=r["amplitude"],
            valid=valid,
        )
    pub_direction.publish(header=ping.header, vector=numpy_to_vector3(r["pos"]))
    pub_debug.publish(
        header=ping.header,
        deltas=r["deltas"].tolist(),
        delta_errors=r["delta_errors"].tolist(),
        fft_sharpness=r["fft_sharpness"],
        heading=heading,
        declination=declination,
    )


if __name__ == "__main__":
    rospy.init_node("hydrophones")
    dist_h = rospy.get_param("~dist_h")
    dist_h4 = rospy.get_param("~dist_h4")
    v_sound = rospy.get_param("~v_sound")
    template_periods = rospy.get_param("~template_periods", 3)
    pub = rospy.Publisher("hydrophones/processed", ProcessedPing, queue_size=10)
    pub_direction = rospy.Publisher(
        "hydrophones/direction", Vector3Stamped, queue_size=10
    )
    pub_debug = rospy.Publisher("hydrophones/debug", Debug, queue_size=10)

    if rospy.myargv()[1:]:
        bag = rosbag.Bag(rospy.myargv()[1])
        for msg in bag.read_messages(["/hydrophones/ping"]):
            process_ping(msg.message)
    else:
        sub = rospy.Subscriber("hydrophones/ping", Ping, process_ping)
        rospy.spin()
