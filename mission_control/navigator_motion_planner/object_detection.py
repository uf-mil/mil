#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from txros import action, util, tf, NodeHandle
import numpy as np
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import Odometry
from navigator_tools import rosmsg_to_numpy, odometry_to_numpy
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef
from navigator_msgs.srv import ScanTheCodeMission, ScanTheCodeMissionResponse
from object_classification import ObjectClassifier

class ObjectDetector:

    def __init__(self, bounding_box_lat_long):
        init_bounding_box = False
        self.ecef_pose = None
        self.pose = None
        self.bounding_box = []
        self.bounding_box_lat_long = bounding_box_lat_long
        self.object_classifier = ObjectClassifier()
        self.odom = rospy.Subscriber('/odom', Odometry, self.set_odom, queue_size=1)
        self.absodom = rospy.Subscriber('/absodom', Odometry, self.set_absodom, queue_size=1)
        self.markers = rospy.Subscriber('/markers_batcave', MarkerArray, self.add_marker)
        self.pub = rospy.Publisher('/rviz/gps', Marker, queue_size=10)
        self.count = 0
        self.set_bouding_box = False


    def set_absodom(self, odom):
        self.absodom.unregister()
        self.ecef_pose = odometry_to_numpy(odom)[0]
        print "absodom"
        if(self.pose is not None and self.ecef_pose is not None):
            self.to_lat_long()
            self.object_classifier.set_pose(self.pose, self.ecef_pose)

            
    def set_odom(self, odom):
        self.odom.unregister()
        self.pose = odometry_to_numpy(odom)[0]
        print "odom"
        if(self.pose is not None and self.ecef_pose is not None):
            self.to_lat_long()
            self.object_classifier.set_pose(self.pose, self.ecef_pose)

    def to_lat_long(self, alt=0):
        for i, ll in enumerate(self.bounding_box_lat_long):
            lat = ll[0]
            lon = ll[1]

            ecef_pos, enu_pos = self.ecef_pose[0], self.pose[0]

            # These functions want radians
            lat, lon = np.radians([lat, lon], dtype=np.float64)
            ecef_vector = ecef_from_latlongheight(lat, lon, alt) - ecef_pos
            enu_vector = enu_from_ecef(ecef_vector, ecef_pos)
            enu_vector[2] = 0  # We don't want to move in the z at all
            pos = np.array(enu_pos + enu_vector)
            self.bounding_box.append(pos)
            self.add_marker_vis(pos, [0,1,0])

        self.set_bouding_box = True
            

    def add_marker_vis(self, pos, colors):
        marker = Marker()
        marker.header.frame_id = "enu";
        marker.header.stamp = rospy.Time.now()
        marker.ns = "my_namespace";
        marker.id = self.count
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = pos[0]
        marker.pose.position.y = pos[1]
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1
        marker.color.a = 1.0
        marker.color.r = colors[0]
        marker.color.g = colors[1]
        marker.color.b = colors[2]
        self.pub.publish(marker);
        self.count+=1



    def add_marker(self, marker_array):
        for marker in marker_array.markers:
            pos = marker.pose.position
            self.object_classifier.classify(pos, marker.scale)
            if(self.in_bounding_box([pos.x,pos.y,pos.z])):
                self.add_marker_vis([pos.x,pos.y,pos.z],[1,0,0])
                self.object_classifier.classify(pos, marker.scale)

    def in_bounding_box(self, pos):
        if(not self.set_bouding_box):
            return
        ab = np.subtract(self.bounding_box[0][:2], self.bounding_box[1][:2])
        ac = np.subtract(self.bounding_box[0][:2], self.bounding_box[2][:2])
        am = np.subtract(self.bounding_box[0][:2], pos[:2])
        if(np.dot(ab,ac) > .1):
            ac = np.subtract(self.bounding_box[0][:2], self.bounding_box[3][:2])
        
        if(0 <= np.dot(ab,am) <= np.dot(ab,ab) and 0 <= np.dot(am,ac) <= np.dot(ac,ac)):
            return True

        return False

def main(args):
  od = ObjectDetector([[29.534647,-82.304280],[29.535011,-82.303323],[29.533803,-82.302639],[29.533400,-82.303577]])
  print "p"
  rospy.init_node('object_detector', anonymous=True)
  print "hgy"
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)