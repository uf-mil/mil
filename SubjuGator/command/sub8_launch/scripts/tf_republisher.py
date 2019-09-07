#!/usr/bin/env python

import rospy
import tf
from mil_msgs.msg import RangeStamped
import mil_misc_tools.text_effects as te


def got_range(msg):
    '''TODO:
        - Make parallel to surface
    '''
    translation = (0.0, 0.0, -msg.range)
    if rospy.Time.now() < rospy.Time(0.5):
        listener.clear()
    t = rospy.Time(0)
    try:
        listener.waitForTransform('/base_link', '/map', t, rospy.Duration(1))
        trans, rot = listener.lookupTransform("/base_link", "/map", t)
        bc.sendTransform(translation, rot, rospy.Time.now(), "/ground", "/dvl")
    except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
        te.fprint(e, title='TF REPUB')


if __name__ == '__main__':
    rospy.init_node('tf_republish')
    rospy.sleep(1)
    bc = tf.TransformBroadcaster()
    listener = tf.TransformListener()
    rospy.sleep(1)

    sub = rospy.Subscriber('/dvl/range', RangeStamped, got_range)

    rospy.spin()
