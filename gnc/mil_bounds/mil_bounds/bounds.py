#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker
from cfg import BoundsConfig
from geometry_msgs.msg import Point
from dynamic_reconfigure.server import Server
from dynamic_reconfigure.client import Client
import numpy as np


def config_to_tuple(config, index):
    return (config['x%d' % index], config['y%d' % index], config['z%d' % index])


def tuple_to_config(tup, config, index):
    config['x%d' % index] = tup[0]
    config['y%d' % index] = tup[1]
    config['z%d' % index] = tup[2]


class BoundsServer(object):
    '''
    Runs the dynamic reconfigure server which implements a global 4 sided
    boundry.
    '''
    def __init__(self):
        self.marker_pub = rospy.Publisher('~visualization', Marker, latch=True, queue_size=1)
        self.server = Server(BoundsConfig, self.update_config)

    def update_config(self, config, level):
        '''
        Callback when a dynamic_reconfigure request arrives.
        Publishes visualization of the boundry.
        '''
        marker = Marker()
        marker.header.frame_id = config.frame
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.5
        marker.color.r = 1
        marker.color.a = 1
        for i in range(1, 5):
            marker.points.append(Point(*config_to_tuple(config, i)))
        marker.points.append(Point(*config_to_tuple(config, 1)))
        self.marker_pub.publish(marker)
        return config


class BoundsClient(Client):
    '''
    Helper class to connect to the bounds server,
    having helper setter/getters with numpy.
    '''
    def __init__(self, server='bounds_server', **kwargs):
        super(BoundsClient, self).__init__(server, **kwargs)

    @staticmethod
    def config_to_numpy(config):
        '''
        Return a 4x3 numpy matrix representing the 4 3D points of the boundry
        '''
        bounds = np.zeros((4, 3), dtype=float)
        for i in range(1, 5):
            bounds[i - 1, :] = np.array(config_to_tuple(config, i))
        return bounds

    @staticmethod
    def numpy_to_config(arr, config):
        '''
        Sets the values of the config from a 4x3 numpy representation
        of the boundry corners
        '''
        for i in range(1, 5):
            tuple_to_config(arr[i - 1], config, i)
        return config

    def set_bounds(self, points, frame='map'):
        '''
        Sets the bounds from a 4x3 numpy array in the specified frame
        '''
        if len(points) != 4:
            return False
        config = self.get_configuration()
        self.numpy_to_config(points, config)
        config['frame'] = frame
        self.update_configuration(config)

    def get_bounds(self):
        '''
        Get the latest bounds from a 4x3 numpy array
        '''
        return self.config_to_numpy(self.get_configuration())

if __name__ == "__main__":
    rospy.init_node("bounds_server")
    bs = BoundsServer()
    rospy.spin()
