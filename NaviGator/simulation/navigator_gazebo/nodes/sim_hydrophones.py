#!/usr/bin/env python
import rospy
from std_srvs.srv import SetBool
from navigator_msgs.srv import FindPinger, FindPingerResponse, SetFrequency


def find_pinger_cb(req):
    # hardcoded for one of the gates
    res = FindPingerResponse()
    res.pinger_position.x = 15.0719184875
    res.pinger_position.y = -15.6615581512
    res.pinger_position.z = 0
    return res


def default_cb(req):
    return {}


if __name__ == '__main__':
    rospy.init_node('hydrophones_sim')
    rospy.Service('/hydrophones/find_pinger', FindPinger, find_pinger_cb)
    rospy.Service('/hydrophones/set_freq', SetFrequency, default_cb)
    rospy.Service('/hydrophones/set_listen', SetBool, default_cb)
    rospy.spin()
