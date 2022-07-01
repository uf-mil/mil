#!/usr/bin/env python3
from typing import Any

import rospy
from navigator_msgs.srv import (
    FindPinger,
    FindPingerRequest,
    FindPingerResponse,
    SetFrequency,
)
from std_srvs.srv import SetBool


def find_pinger_cb(_: FindPingerRequest) -> FindPingerResponse:
    # hardcoded for one of the gates
    res = FindPingerResponse()
    res.pinger_position.x = 15.0719184875
    res.pinger_position.y = -15.6615581512
    res.pinger_position.z = 0
    return res


def default_cb(_: Any) -> dict:
    return {}


if __name__ == "__main__":
    rospy.init_node("hydrophones_sim")
    rospy.Service("/hydrophones/find_pinger", FindPinger, find_pinger_cb)
    rospy.Service("/hydrophones/set_freq", SetFrequency, default_cb)
    rospy.Service("/hydrophones/set_listen", SetBool, default_cb)
    rospy.spin()
