#!/usr/bin/env python

import rospy
from mil_ros_tools.srv import BaggerCommands

'''
Bags the current stream of topics defined in the
online_bagger using the service '/bagging server'
will return a success message along with the list of topics that were bagged

For example:
    bag_status = bagging_server(#, 'units')     will bag # units as long as units is seconds/minutes
    bag_status = bagging_server()               will bag everything
    bag_status = bagging_server(10, 'seconds')  will bag the last 10 seconds
    bag_status = bagging_server(5, 'minutes')   will bag last 5 minutes
    bag_status = bagging_server(#, 'status')    will display the current status - No bagging:

    Units for seconds may be 's', 'sec', 'second', 'seconds'
    Units for minutes may be 'm', 'min', 'minute', 'minutes'
    Units for # is any float number

In Particular the current status will be:
- Number of Topics Subscribed To
- Name of Topics Subscribed To
- Total Message Count Across all Topics
- Current Memory Usage

'''
def online_bagging_client(amount=0, unit=''):
    rospy.wait_for_service('/online_bagger/dump')
    try:
        bagging_server = rospy.ServiceProxy('/online_bagger/dump', BaggerCommands)
        bag_status = bagging_server(amount, unit)
    except rospy.ServiceException, e:
        print "/online_bagger service failed: %s" % e


# online_bagging_client(30, 'm')
