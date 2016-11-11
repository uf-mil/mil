#!/usr/bin/env python
import numpy as np

import rospy
import navigator_tools
from twisted.internet import defer

from navigator_tools.cfg import BoundsConfig
from dynamic_reconfigure.server import Server

from navigator_msgs.srv import Bounds, BoundsResponse, \
                               CoordinateConversion, CoordinateConversionRequest


class BoundsServer(object):
    def __init__(self):
        self.convert = rospy.ServiceProxy('/convert', CoordinateConversion)

        self.enforce = False
        self.bounds = []

        rospy.set_param("/bounds/enu/", [[0,0],[0,0],[0,0],[0,0]])
        
        rospy.set_param("/bounds/enforce", self.enforce)
        #self.convert = Converter()
        Server(BoundsConfig, self.update_config)
        rospy.Service('/get_bounds', Bounds, self.got_request)

    def destringify(self, lat_long_string):
        if any(c.isalpha() for c in lat_long_string):
            return False

        floats = map(float, lat_long_string.replace('(', '').replace(')', '').split(',')) 

        if len(floats) != 3:
            return False
        
        return floats

    def update_config(self, config, level):
        self.enforce = config['enforce']
        lla_bounds_str = [config['lla_1'] + ", 0", config['lla_2'] + ", 0", 
                          config['lla_3'] + ", 0", config['lla_4'] + ", 0"]
        lla_bounds = map(self.destringify, lla_bounds_str)
        
        # If any returned false, don't update
        if all(lla_bounds):
            self.bounds = [self.convert(CoordinateConversionRequest(frame="lla", point=lla)) for lla in lla_bounds]
            config['enu_1_lat'] = self.bounds[0].enu[0]
            config['enu_1_long'] = self.bounds[0].enu[1]
            
            config['enu_2_lat'] = self.bounds[1].enu[0]
            config['enu_2_long'] = self.bounds[1].enu[1]

            config['enu_3_lat'] = self.bounds[2].enu[0]
            config['enu_3_long'] = self.bounds[2].enu[1]

            config['enu_4_lat'] = self.bounds[3].enu[0]
            config['enu_4_long'] = self.bounds[3].enu[1]
            
            rospy.set_param("/bounds/enu/", map(lambda bound: bound.enu[:2], self.bounds))
        else:
            rospy.set_param("/bounds/enu/", [[0,0],[0,0],[0,0],[0,0]])

        rospy.set_param("/bounds/enforce", self.enforce)
        return config

    def got_request(self, req):
        to_frame = "enu" if req.to_frame == '' else req.to_frame
        
        resp = BoundsResponse(enforce=False)

        bounds = [navigator_tools.numpy_to_point(getattr(response, to_frame)) for response in self.bounds]

        resp.enforce = self.enforce
        resp.bounds = bounds        

        return resp

if __name__ == "__main__":
    rospy.init_node("bounds_server")
    bs = BoundsServer()
    rospy.spin()
