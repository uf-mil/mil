#!/usr/bin/env python
import rospy
from navigator_tools.cfg import BoundsConfig
from dynamic_reconfigure.server import Server
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
        for i in (1, 2, 3, 4):
            res = self.convert(CoordinateConversionRequest(
                frame="lla",
                point=(config['lat%d' % i], config['long%d' % i], 0.0)))
            config['x%d' % i] = res.enu[0]
            config['y%d' % i] = res.enu[1]

    def update_lla(self, config):
        '''
        Given a bounds config, update the LLA bounds by
        converting the ENU bounds to LLA
        '''
        for i in (1, 2, 3, 4):
            res = self.convert(CoordinateConversionRequest(
                frame="enu",
                point=(config['x%d' % i], config['y%d' % i], 0.0)))
            config['lat%d' % i] = res.lla[0]
            config['long%d' % i] = res.lla[1]

    def update_config(self, config, level):
        '''
        Callback when a dynamic_reconfigure request arrives.
        Convert ENU/LLA as needed.
        '''
        if level == -1 or level == 0 or level & 2 == 2:  # If default or LLA changed, convert to ENU
            self.update_enu(config)
        elif level & 4 == 4:  # Otherwise if ENU is changed, update LLA
            self.update_lla(config)
        return config


if __name__ == "__main__":
    rospy.init_node("bounds_server")
    bs = BoundsServer()
    rospy.spin()
