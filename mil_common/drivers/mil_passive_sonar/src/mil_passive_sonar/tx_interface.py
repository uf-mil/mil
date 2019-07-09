#!/usr/bin/env python
from txros import util
from twisted.internet import defer
from geometry_msgs.msg import Vector3Stamped, PointStamped
from std_srvs.srv import SetBool, SetBoolRequest, Trigger, TriggerRequest


class TxHydrophonesClient(object):
    '''
    TXROS abstraction for interacting with the nodes in this package
    '''
    def __init__(self, nh):
        '''
        Construct a client. nh is a txros nodehandle object
        '''
        self._direction_sub = nh.subscribe("/hydrophones/direction", Vector3Stamped, callback=self.heading_cb)
        self._enable_srv = nh.get_service_client("/multilateration/enable", SetBool)
        self._reset_srv = nh.get_service_client("/multilateration/reset", Trigger)
        self._position_sub = nh.subscribe("/hydrophones/position", PointStamped)
        self.dir_callback = None

    def get_direction(self):
        '''
        Get the next processed direction to the pinger
        '''
        return self._direction_sub.get_next_message()

    def get_position(self):
        '''
        Get the next processed position of the pinger
        '''
        return self._position_sub.get_next_message()

    def get_last_position(self):
        '''
        Get the last processed position of the pinger
        '''
        return self._position_sub.get_last_message()

    def enable(self):
        '''
        Enable listening to pings for position estimation
        '''
        return self._enable_srv(SetBoolRequest(data=True))

    def reset(self):
        '''
        Reset the position estimation of the pinger
        '''
        return self._reset_srv(TriggerRequest())

    def disable(self):
        '''
        Disable listening to pings for position estimation
        '''
        return self._enable_srv(SetBoolRequest(data=False))

    def heading_cb(self, heading_msg):
        '''
        Callback for pings recieved
        '''
        if self.dir_callback is not None:
            self.dir_callback(heading_msg)

    def set_callback(self, cb):
        '''
        Set a callback for when a ping is received
        '''
        self.dir_callback = cb
