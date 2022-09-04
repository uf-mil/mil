#!/usr/bin/env python3
import numpy as np
import txros
from mil_misc_tools import ThrowingArgumentParser
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
from std_srvs.srv import SetBoolRequest

from .navigator import Navigator


class Wildlife(Navigator):

    @classmethod
    def init(cls):
        pass

    @txros.util.cancellableInlineCallbacks
    def run(self, parameters):
        # Go to autonomous mode
        yield self.set_classifier_enabled.wait_for_service()
        yield self.set_classifier_enabled(SetBoolRequest(data=True))
        yield self.nh.sleep(4)
        yield self.change_wrench("autonomous")
        
        try:
            t1 = yield self.get_sorted_objects("mb_marker_buoy_red", n=1)
            t1 = t1[1][0]
        except Exception as e:
            print("could not find stc_platform")
            # get all pcodar objects
            try:
                print("check for any objects")
                t1 = yield self.get_sorted_objects(name="UNKNOWN", n=-1)
                t1 = t1[1][0]
            # if no pcodar objects, drive forward
            except Exception as e:
                print("literally no objects?")
                yield self.move.forward(10).go()
                # get first pcodar objects
                t1 = yield self.get_sorted_objects(name="UNKNOWN", n=-1)
                t1 = t1[1][0]
                # if still no pcodar objects, guess RGB and exit mission
            # go to nearest obj to get better data on that obj


            print("going to nearest small object")

        points = self.move.d_spiral_point(t1,5,8,1,"ccw")
        for p in points:
            yield p.go()

