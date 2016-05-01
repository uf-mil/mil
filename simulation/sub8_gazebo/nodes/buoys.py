#!/usr/bin/env python
import rospy
import tf

from gazebo_msgs.msg import ContactsState
from std_msgs.msg import String

import numpy as np


def check_contact(msg, buoy_pub):
    if len(msg.states) == 0:
        # If there is no impact don't worry about it.
        return

    sub_name = "sub8::base_link::box_collision"

    object_name = None
    for state in msg.states:
        # It seems like collisions are in alpabetical order - so check both.
        if state.collision1_name == sub_name:
            object_name = state.collision2_name.split('::')[0]
            break
        if state.collision2_name == sub_name:
            object_name = state.collision1_name.split('::')[0]
            break

    if object_name is None:
        return

    buoy_pub.publish(object_name)

rospy.init_node('torpedo_manager')
buoy_pub = rospy.Publisher('/gazebo/buoy_bumping', String, queue_size=1)
rospy.Subscriber('/contact_bumper', ContactsState, check_contact, buoy_pub, queue_size=10)
rospy.spin()
