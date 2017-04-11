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
  def fix_topic(self, topic):
      for k, t in self.topic_map.iteritems():
          if topic.find(k) == 0:
              return t+topic[len(k):]
      return topic

  def fix_tf(self, tf):
      for i, t in enumerate(tf.transforms):
          tf.transforms[i].header.frame_id = self.fix_frame(tf.transforms[i].header.frame_id)
          tf.transforms[i].child_frame_id = self.fix_frame(tf.transforms[i].child_frame_id)
      return tf

  def fix_frame(self, frame):
      fixed_frame = self.frame_map.get(frame)
      return fixed_frame if fixed_frame != None else frame
      
  def fix_bag(self, infile, outfile=None):
      if outfile == None:
          split = infile.rsplit('.bag', 1)
          outfile = split[0]+'_fixed.bag'
      out = rosbag.Bag(outfile, 'w')
      bag = rosbag.Bag(infile)
      for topic, msg, time in bag.read_messages():
        topic = self.fix_topic(topic)
        if hasattr(msg, 'header') and msg.header._type == 'std_msgs/Header':
          msg.header.frame_id = self.fix_frame(msg.header.frame_id)
        if msg._type == 'tf2_msgs/TFMessage' or msg._type == 'tf/tfMessage':
          msg = self.fix_tf(msg)
        out.write(topic, msg, time)
      out.flush()
    
  def fix_strings(self, strings):
      if type(strings) == dict:
          return strings
      assert type(strings) == list
      d = {}
      for s in strings:
        split = s.split(':')
        assert len(split) == 2
        d[split[0]] = split[1]
      return d
        
  def __init__(self, topic_map={}, frame_map={}):
      self.topic_map = self.fix_strings(topic_map)
      self.frame_map = self.fix_strings(frame_map)
      print self.topic_map
      print self.frame_map
      #self.fix_bag()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Fix bag topics/frame_ids')
    parser.add_argument('--in', '-i', dest='infile', type=str, required=True,
                        help='Bag to read and fix')
    parser.add_argument('--out', '-o', type=str, dest='outfile',
                        help='Bag to output with fixed content, defaults to INPUTFILENAME_fixed.bag')
    parser.add_argument('--topics',required=False, nargs='+', type=str, metavar='oldtopic:newtopic',
                          help="list of topics to remap, seperated by a colon, ex /down_cam/:/camera/down/ /my_odom:/odom\
                          If ends in a slash (ex: /cameras/:/cams/, all topics after slash will be remaped")
    parser.add_argument('--frames',required=False, nargs='+', type=str, metavar='oldframe:newframe',
                          help="list of frame_ids in headers to be maped, ex: my_camera:cam_front")
    args = parser.parse_args()
    fixer = BagFixer(args.topics, args.frames)
    fixer.fix_bag(args.infile, args.outfile)
    #fixer = BagFixer(args.inbag, args.outbag)
  
  
