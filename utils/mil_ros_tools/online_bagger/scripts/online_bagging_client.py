#!/usr/bin/env python

import rospy
from std_srvs.srv import SetBool

'''
Bags the current stream of topics defined in the
online_bagger using the service '/bagging server'
will return a success message along with the list of topics that were bagged
'''
def online_bagging_client():
    rospy.wait_for_service('/online_bagger/dump')
    try:
        bagging_server = rospy.ServiceProxy('/online_bagger/dump', SetBool)
        bag_status = bagging_server(True)
    except rospy.ServiceException, e:
        print "/online_bagger service failed: %s" % e


online_bagging_client()
