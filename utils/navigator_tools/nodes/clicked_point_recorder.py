#!/usr/bin/env python

"""
Listens to RVIZ clicked points, storing points in a csv file

Usage: rosrun navigator_tools clicked_point_recorder.py
"""

import rospy
import datetime
import csv
from geometry_msgs.msg import PointStamped
class ClickedPointRecorder:
  def __init__(self):
      self.point_sub = rospy.Subscriber("/clicked_point", PointStamped, self.point_cb)
      self.points = []

  def point_to_dict(self, point):
      return { 'seq': point.header.seq,
               'secs' : point.header.stamp.secs,
               'nsecs' : point.header.stamp.nsecs,
               'frame_id' : point.header.frame_id,
               'x' : point.point.x,
               'y': point.point.y,
               'z': point.point.z}

  def write_file(self):
      time = datetime.datetime.now().strftime("%Y-%m-%d-%H-%M-%S")
      filename = 'clickedpoints{}.csv'.format(time)
      with open(filename ,'wx') as csvfile:
          fieldnames = ['seq','secs','nsecs','frame_id','x','y','z']
          writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
          writer.writeheader()
          for p in self.points:
              d = self.point_to_dict(p)
              writer.writerow(d)
      rospy.loginfo("Writing points to {}".format(filename))

  def point_cb(self,point):
      rospy.loginfo("Received new point: {}".format(point))     
      self.points.append(point)

if __name__ == '__main__':
    rospy.init_node('clicked_point_recorder')
    recorder = ClickedPointRecorder()
    def shutdown_cb():
      recorder.write_file()
    rospy.on_shutdown(shutdown_cb)
    rospy.spin()
