#!/usr/bin/env python
import rospy
from navigator_tools.cfg import BoundsConfig
from dynamic_reconfigure.server import Server
from geometry_msgs.msg import Point
from navigator_msgs.srv import CoordinateConversion, CoordinateConversionRequest
from mil_tools import wait_for_service


class BoundsServer(object):
    '''
    Runs the dynamic reconfigure server which implements a global 4 rectangular
    boundry outside of which NaviGator will not move.
    '''
    def __init__(self):
        self.convert = rospy.ServiceProxy('/convert', CoordinateConversion)
        # Ensure conversion is available before accepting changes
        wait_for_service(self.convert, warn_msg='Waiting for conversion service')
        self.server = Server(BoundsConfig, self.update_config)

    def update_enu(self, config):
        '''
        Given a bounds config, update the ENU bounds by
        converting the LLA bounds to ENU
        '''
        req = CoordinateConversionRequest()
        req.frame = req.LLA
        req.to_frame = req.ENU
        req.points = [Point(config['lat%d' % i], config['long%d' % i], 0) for i in (1, 2, 3, 4)]
        res = self.convert(req)
        for i in (1, 2, 3, 4):
            config['x%d' % i] = res.converted[i - 1].x
            config['y%d' % i] = res.converted[i - 1].y

    def update_lla(self, config):
        '''
        Given a bounds config, update the LLA bounds by
        converting the ENU bounds to LLA
        '''
        req = CoordinateConversionRequest()
        req.frame = req.ENU
        req.to_frame = req.LLA
        req.points = [Point(config['x%d' % i], config['y%d' % i], 0) for i in (1, 2, 3, 4)]
        res = self.convert(req)
        for i in (1, 2, 3, 4):
            config['lat%d' % i] = res.converted[i - 1].x
            config['long%d' % i] = res.converted[i - 1].y

    def update_config(self, config, level):
        '''
        Callback when a dynamic_reconfigure request arrives.
        Convert ENU/LLA as needed.
        '''
        if level == -1 or level == 0 or level & 2 == 2:  # If default or LLA changed, convert to ENU
            rospy.loginfo('Updating bounds with LLA points')
            self.update_enu(config)
        elif level & 4 == 4:  # Otherwise if ENU is changed, update LLA
            rospy.loginfo('Updating bounds with ENU points')
            self.update_lla(config)
        return config


if __name__ == "__main__":
    rospy.init_node("bounds_server")
    bs = BoundsServer()
    rospy.spin()
