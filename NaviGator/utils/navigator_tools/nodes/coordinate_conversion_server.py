#!/usr/bin/env python
from __future__ import division
import rospy
import numpy as np
from mil_tools import rosmsg_to_numpy, odometry_to_numpy, numpy_to_point, numpy_to_points
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef_tf, latlongheight_from_ecef
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PointStamped
from navigator_msgs.srv import CoordinateConversion, CoordinateConversionResponse
import message_filters


class EarthCoordinateConverter(object):
    '''
    Node to perform conversion between points in ENU (East North Up),
    LLA (Longitude Latitude Altitude) and ECEF (earth centered earth fixed).

    Subscribes to odom in ENU and absodom in ECEF to perform these conversions via a service call.

    Also publishes the current LLA point at each syncronized message of odom and absodom.
    '''
    FRAMES = ['enu', 'ecef', 'lla']

    def __init__(self):
        self.lla_pub = rospy.Publisher('lla', PointStamped, queue_size=5)
        odom_sub = message_filters.Subscriber('odom', Odometry)
        abs_odom_sub = message_filters.Subscriber('absodom', Odometry)
        self.first = True
        self.stamp = None
        self.enu_pos = None
        self.ecef_pos = None
        self.ts = message_filters.TimeSynchronizer([odom_sub, abs_odom_sub], 10)
        self.ts.registerCallback(self.odom_sync_cb)

    def odom_sync_cb(self, odom, absodom):
        '''
        Called each time and odom and absodom arrive with the same stamp. Caches these
        values for later conversions and publishes current LLA.
        '''
        if self.first:  # Advertise convert service on first odom/absodom pair
            self.convert_service = rospy.Service('convert', CoordinateConversion, self.convert_cb)
            self.first = False
        self.stamp = odom.header.stamp
        self.enu_pos = odometry_to_numpy(odom)[0][0]
        self.ecef_pos = odometry_to_numpy(absodom)[0][0]
        self.last_pair = (odom, absodom)
        self.publish_lla()

    def publish_lla(self):
        '''
        Publish a point message with the current LLA computed from an odom and absodom message.
        '''
        _, _, lla = self.enu(self.enu_pos)
        lla_msg = PointStamped()
        lla_msg.header.frame_id = 'lla'
        lla_msg.header.stamp = self.stamp
        lla_msg.point = numpy_to_point(lla)
        self.lla_pub.publish(lla_msg)

    def convert_cb(self, req):
        '''
        Callback for conversion service. Converts all points
        from the in_frame to to_frame.
        '''
        if req.frame not in self.FRAMES:
            return CoordinateConversionResponse(message='in_frame not valid')
        if req.to_frame not in self.FRAMES:
            return CoordinateConversionResponse(message='to_frame not valid')
        idx = self.FRAMES.index(req.to_frame)
        converted = np.zeros((len(req.points), 3))
        func = getattr(self, req.frame)
        for i in range(len(req.points)):
            converted[i] = func(rosmsg_to_numpy(req.points[i]))[idx]
        return CoordinateConversionResponse(converted=numpy_to_points(converted))

    def lla(self, point):
        '''
        Produces enu, ecef, and lla from an lla point.
        '''
        lla = point
        ecef = ecef_from_latlongheight(*np.radians(lla))
        ecef_vector = ecef - self.ecef_pos
        enu_vector = enu_from_ecef_tf(self.ecef_pos).dot(ecef_vector)
        enu = enu_vector + self.enu_pos
        return enu, ecef, lla

    def ecef(self, point):
        '''
        Produces enu, ecef, and lla from ecef point.
        '''
        ecef = point
        lla = np.degrees(latlongheight_from_ecef(ecef))
        ecef_vector = ecef - self.ecef_pos
        enu_vector = enu_from_ecef_tf(self.ecef_pos).dot(ecef_vector)
        enu = enu_vector + self.enu_pos
        return enu, ecef, lla

    def enu(self, point):
        '''
        Produces enu, ecef, and lla from enu point.
        '''
        enu = point
        enu_vector = enu - self.enu_pos
        ecef_vector = enu_from_ecef_tf(self.ecef_pos).T.dot(enu_vector)
        ecef = ecef_vector + self.ecef_pos
        lla = np.degrees(latlongheight_from_ecef(ecef))
        return enu, ecef, lla


if __name__ == "__main__":
    rospy.init_node('earth_coordinate_converter')
    c = EarthCoordinateConverter()
    rospy.spin()
