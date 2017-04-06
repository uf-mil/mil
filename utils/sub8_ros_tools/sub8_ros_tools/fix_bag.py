#!/usr/bin/env python
import argparse
import rosbag
import rospy
from sensor_msgs.msg import CameraInfo
from roslib.message import get_message_class

class BagFixer():
  '''
  Dictionary of topics to remap. If ends in /, remaps everything after
  Otherwise, topic must match exactly
  '''
  TOPIC_MAP = {
    "/down/"             : "/camera/down/left/",
    "/down/left/"        : "/camera/down/left/",
    "/down_camera/"      : "/camera/down/left/",
    "/stereo/right/"     : "/camera/front/right/",
    "/stereo/left/"      : "/camera/front/left/",
    "/forward_camera/"   : "/camera/front/"
  }
  FRAME_MAP = {
    "downward"           : "down_left_cam",
    "down_camera"        : "down_left_cam",
    "stereo_front"       : "front_stereo",
    "/stereo/right_cam"  : "front_right_cam",
    "/stereo/left_cam"   : "front_left_cam",
    "forward_camera"     : "front_stereo"
  }

  def fix_topic(self, topic):
      for k, t in BagFixer.TOPIC_MAP.iteritems():
          if topic.find(k) == 0:
            return t+topic[len(k):]
      return topic

  def fix_tf(self, tf):
    for i, t in enumerate(tf.transforms):
      tf.transforms[i].header.frame_id = self.fix_frame(tf.transforms[i].header.frame_id)
      tf.transforms[i].child_frame_id = self.fix_frame(tf.transforms[i].child_frame_id)
    return tf

  def fix_frame(self, frame):
      fixed_frame = BagFixer.FRAME_MAP.get(frame)
      return fixed_frame if fixed_frame != None else frame
      
  def fix_bag(self):
      out = rosbag.Bag(self.outbag, 'w')
      bag = rosbag.Bag(self.inbag)
      for topic, msg, time in bag.read_messages():
        topic = self.fix_topic(topic)
        if hasattr(msg, 'header') and msg.header._type == 'std_msgs/Header':
          msg.header.frame_id = self.fix_frame(msg.header.frame_id)
        if msg._type == 'tf2_msgs/TFMessage' or msg._type == 'tf/tfMessage':
          msg = self.fix_tf(msg)
        out.write(topic, msg, time)
      out.flush()
    

  def __init__(self, inbag, outbag):
    self.inbag = inbag
    if outbag == 'DEFAULT':
      split = inbag.rsplit('.bag', 1)
      outbag = split[0]+'_fixed.bag'
    self.outbag = outbag
    self.fix_bag()



if __name__ == "__main__":
  parser = argparse.ArgumentParser(description='Fix bag topics/frame_ids')
  parser.add_argument('inbag', type=str,
                      help='Bag to read and fix')
  parser.add_argument('-o', type=str, dest='outbag', default='DEFAULT',
                      help='Bag to output with fixed content, defaults to INPUTFILENAME_fixed.bag')
  args = parser.parse_args()
  fixer = BagFixer(args.inbag, args.outbag)
  
  
