#!/usr/bin/env python
from geometry_msgs.msg import Vector3Stamped, PointStamped
from std_srvs.srv import SetBool, SetBoolRequest


class TxHydrophonesClient(object):
    '''
    TXROS abstraction for interacting with the nodes in this package
    '''
    def __init__(self, nh):
        '''
        Construct a client. nh is a txros nodehandle object
        '''
        self._direction_sub = nh.subscribe("/hydrophones/direction", Vector3Stamped)
        self._enable_srv = nh.get_service_client("/multilateration/enable", SetBool)
        self._position_sub = nh.subscribe("/hydrophones/position", PointStamped)

    def get_direction(self):
        '''
        Get the next processed direction to the pinger
        '''
        return self._direction_sub.get_next_message()

    def get_position(self):
        '''
        Get the next processed position to the pinger
        '''
        return self._position_sub.get_next_message()

    def enable(self):
        '''
        Enable listening to pings for position estimation
        '''
        return self._enable_srv(SetBoolRequest(data=True))

    def disable(self):
        '''
        Disable listening to pings for position estimation
        '''
        return self._enable_srv(SetBoolRequest(data=False))
