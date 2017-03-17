#!/usr/bin/env python
from __future__ import division

import txros
import numpy as np
import navigator_tools
from twisted.internet import defer
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef_tf, latlongheight_from_ecef

from nav_msgs.msg import Odometry
from navigator_msgs.srv import CoordinateConversion, CoordinateConversionResponse

class Converter(object):
    @txros.util.cancellableInlineCallbacks
    def init(self, nh, advertise_service=False):
        self.nh = nh
        self.odom_sub = yield self.nh.subscribe('odom', Odometry)
        self.abs_odom_sub = yield self.nh.subscribe('absodom', Odometry)

        if advertise_service:
            yield self.nh.advertise_service('/convert', CoordinateConversion, self.request)

    def __getattr__(self, attr):
        print "Frame '{}' not found!".format(attr)
        return self.not_found

    def not_found(self):
        return [[np.nan, np.nan, np.nan]] * 3

    @txros.util.cancellableInlineCallbacks
    def request(self, srv):
        self.point = np.array(srv.point)

        last_odom = yield self.odom_sub.get_last_message()
        last_absodom = yield self.abs_odom_sub.get_last_message()

        if last_odom is None or last_absodom is None:
            defer.returnValue(CoordinateConversionResponse())

        self.enu_pos = navigator_tools.odometry_to_numpy(last_odom)[0][0]
        self.ecef_pos = navigator_tools.odometry_to_numpy(last_absodom)[0][0]

        enu, ecef, lla = getattr(self, srv.frame)()

        distance = np.linalg.norm(enu - self.enu_pos)
        defer.returnValue(CoordinateConversionResponse(enu=enu, ecef=ecef, lla=lla, distance=distance))

    def lla(self):
        lla = self.point

        # to ecef
        ecef = ecef_from_latlongheight(*np.radians(lla))

        # to enu
        ecef_vector = ecef - self.ecef_pos
        enu_vector = enu_from_ecef_tf(self.ecef_pos).dot(ecef_vector)
        enu = enu_vector + self.enu_pos

        return enu, ecef, lla

    def ecef(self):
        ecef = self.point

        # to lla
        lla = np.degrees(latlongheight_from_ecef(ecef))

        # to enu
        ecef_vector = ecef - self.ecef_pos
        enu_vector = enu_from_ecef_tf(self.ecef_pos).dot(ecef_vector)
        enu = enu_vector + self.enu_pos

        return enu, ecef, lla

    def enu(self):
        enu = self.point

        # to ecef
        enu_vector = enu - self.enu_pos
        ecef_vector = enu_from_ecef_tf(self.ecef_pos).T.dot(enu_vector)
        ecef = ecef_vector + self.ecef_pos

        # to lla
        lla = np.degrees(latlongheight_from_ecef(ecef))

        return enu, ecef, lla


@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("coordinate_conversion_server")
    c = Converter()
    yield c.init(nh, advertise_service=True)

    yield defer.Deferred() # never exit

if __name__ == "__main__":
    txros.util.launch_main(main)
