#!/usr/bin/env python
"""
Converts bags to labelme images, or labelme annotations to bags.

"""

import argparse
import yaml
import xml.etree.ElementTree
import os
import rosbag
import rospy
import cv2
import genpy
from cv_bridge import CvBridge
from mil_msgs.msg import LabeledObjects, LabeledObject

class BagToLabelMe():
  def __init__(self, config, labelme_dir):
      if not os.path.isdir(labelme_dir):
          raise Exception("Labelme directory {} does not exsist".format(labelme_dir))

      config_file = open(config, 'r')
      self.config = yaml.load(config_file)
      self._verify_yaml()
      self.bridge = CvBridge()
      self.labelme_dir = labelme_dir

  def _verify_yaml(self):
      if not 'bags' in self.config:
          raise Exception("Config file does not contain 'bags' attribute")
      for bag in self.config['bags']:
          if not 'file' in bag:
              raise Exception("Bag does not contain 'file' attribute {}".format(bag))
          if not 'segments' in bag:
              raise Exception("Bag does not contain 'segments' attribute {}".format(bag))
          for segment in bag['segments']:
              if not 'start' in segment:
                  raise Exception("Segment does not contain 'start' attribute {}".format(segment))
              if not 'stop' in segment:
                  raise Exception("Segment does not contain 'stop' attribute {}".format(segment))
              if not 'topic' in segment:
                  raise Exception("Segment does not contain 'topic' attribute {}".format(segment))
              if not 'name' in segment:
                  raise Exception("Segment does not contain 'name' attribute {}".format(segment))
              if not 'freq' in segment:
                  raise Exception("Segment does not contain 'freq' attribute {}".format(segment))

  def _read_bag(self, bagfile, segments):
      bag = rosbag.Bag(bagfile)
      _, _, first_time = bag.read_messages().next()
      for segment in segments:
          print "\tProccessing Segment '{}' with topic '{}'".format(segment['name'], segment['topic'])
          path = os.path.join(self.labelme_dir, 'Images', segment['name'], segment['topic'].replace('/', '@'))
          if not os.path.exists(path):
              os.makedirs(path)
          start = first_time + rospy.Duration(segment['start'])
          stop = first_time + rospy.Duration(segment['stop'])
          interval = rospy.Duration(1.0 / segment['freq'])
          print "\t\tSaving Images from duration {} to {} every {} seconds".format(segment['start'], segment['stop'], interval.to_sec())
          next_time = start
          for topic, msg, time in bag.read_messages(topics=[segment['topic']], start_time = start, end_time = stop):
              if time >= next_time:
                  img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                  filename = os.path.join(path, str(msg.header.stamp) + '.jpg')
                  #print "WRITING", filename
                  cv2.imwrite(filename, img)
                  next_time = next_time + interval

  def read_bags(self):
      for bag in self.config['bags']:
          print "Opening {}".format(bag['file'])
          self._read_bag(bag['file'], bag['segments'])

  def print_report(self):
      print "Doing report"

  def _parse_label_xml(self, filename):
      msg = LabeledObjects()
      e = xml.etree.ElementTree.parse(filename).getroot()
      #TODO: parse into labeled object
      return msg

  def _get_labels(self, bag_config):
      labels = {}
      for segment in bag_config['segments']:
          labels[segment['topic']] = {}
      for segment in bag_config['segments']:
          path = os.path.join(self.labelme_dir, 'Annotations', segment['name'], segment['topic'].replace('/', '@'))
          print "\t\tExtracting labels from {}".format(path)
          for xmlfile in os.listdir(path):
              stamp = xmlfile.split('.xml', 1)
              assert len(stamp) == 2
              print "\t\t\tParsing File '{}'".format(stamp[0])
              labels[segment['topic']][stamp[0]] = os.path.join(path, xmlfile)
      return labels
      #~ {'topic1' : {'stamp1': labels, 'stamp2' :labels}
       #~ 'topic2': {'stamp1': labels}}

  def _extract_labels_bag(self, bag_config, out):
      label_files = self._get_labels(bag_config)
      bag = rosbag.Bag(bag_config['file'])
      out = rosbag.Bag(out, mode='w')
      _, _, first_time = bag.read_messages().next()
      for topic, msg, t in bag.read_messages():
        if msg._type == 'sensor_msgs/Image':
            if topic in label_files:
              if str(msg.header.stamp) in label_files[topic]:
                  labels = self._parse_label_xml(label_files[topic][str(msg.header.stamp)])
                  labels.header = msg.header
                  print "GOT LABEL {}".format(label_files[topic][str(msg.header.stamp)])
                  #out.write(topic+'labels', labels, t)
        #out.write(topic, msg, t)

  def extract_labels(self):
      for bag in self.config['bags']:
          print "\tExtracting labels for {}".format(bag['file'])
          # TODO: give output bag a default name like INFILE_labeled.bag
          outfile = bag['labeled_file']
          self._extract_labels_bag(bag, outfile)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert rosbags to labelme')
    parser.add_argument('config', type=str,
                        help='YAML file to read')
    parser.add_argument('--labelme-dir', '-d', dest='dir',type=str, required=True,
                        help='root directory of labelme instalation')
    parser.add_argument('--extract', '-e', dest='extract_labels', action='store_true',
                        help='Read annotations from labelme, inserting them into a new bag')
    parser.add_argument('--report', '-r', dest='do_report', action='store_true',
                        help='Read annotations from labelme and produces a report on labeling coverage')
    args = parser.parse_args()
    bag_to_labelme = BagToLabelMe(args.config, args.dir)
    if args.extract_labels:
        bag_to_labelme.extract_labels()
    elif args.do_report:
        bag_to_labelme.print_report()
    else:
        bag_to_labelme.read_bags()
