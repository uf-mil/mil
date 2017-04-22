#!/usr/bin/env python

"""
Converts bags to labelme images, or labelme annotations to bags.

Uses a YAML file to specify what times and topics in a set of bag files
will be used to produce images which are places in a corosponding
folder in a LabelMe instalation.

When run with the -e flag, uses same YAML file to read LabelMe annotations
and place them as a ros message at syncronized times with the images
they corospond to.

With the -r flag, a report will be generated based on the specified YAML
about how many images are labeled in LabelMe

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
from geometry_msgs.msg import Point
from mil_msgs.msg import LabeledObjects, LabeledObject

class BagToLabelMe():
  """
  Interfaces between ROS bags and LabelMe images.
  
  Can be used to insert images from ROS bags into LabelMe instalation
  or extract LabelMe annotations into a ROS bag
  
  TODO: improve verbosity, disable print statements with toggle
  """
  def __init__(self, config, labelme_dir):
      """
      Generates class that can be used to insert or extract images to/from LabelMe
      
      config: path to YAML file following format of the example yaml file in this package
      labelme_dir: root directory of labelme instalation (should contain Images and Annotations directories)
      """
      if not os.path.isdir(labelme_dir):
          raise Exception("Labelme directory {} does not exsist".format(labelme_dir))

      config_file = open(config, 'r')
      self.config = yaml.load(config_file)
      self._verify_yaml()
      self.bridge = CvBridge()
      self.labelme_dir = labelme_dir

  def read_bags(self):
      """
      Crawls through all bags in config YAML, placing images from the specified
      topics and the specified frequency into corosponding folders within
      Images of the labelme instalation.
      
      These folders will be created as follows if not already:
      LABELME_DIR/Images/SEGEMENT_NAME/TOPIC_NAME
      where TOPIC_NAME replaces all / with @ for compatibility with filesystems.
      """
      for bag in self.config['bags']:
          print "Opening {}".format(bag['file'])
          self._read_bag(bag['file'], bag['segments'])

  def extract_labels(self):
      """
      Finds labeled XML files in LabelMe directories determined from the
      config file, placing parsed annotation data as ros messages in a new bag
      """
      print "Extracting labels for all bags in config"
      for bag in self.config['bags']:
          print "\tExtracting LabelMe annotations for {}".format(bag['file'])
          # TODO: give output bag a default name like INFILE_labeled.bag
          outfile = bag['labeled_file']
          self._extract_labels_bag(bag, outfile)

  def print_report(self):
      """
      Compares how many images are present in each LabelMe directory affected
      by the config YAML to the number of annotated XML files produced by
      LabelMe in each of these directories.
      
      Prints out these counts in percentages by segement, bag, and overall
      """
      print "Generating completion report for all bags in config"
      total_xml_count = 0
      total_img_count = 0
      for bag in self.config['bags']:
          print "\tGenerating completion report for {}".format(bag['file'])
          x, i = self._print_bag_report(bag)
          total_xml_count += x
          total_img_count += i
      print "{}/{} TOTAL images labeled ({}%)".format(total_xml_count, total_img_count, round(100.0*total_xml_count/total_img_count,2))

  def _verify_yaml(self):
      """
      Called by constructor to ensure YAML follows correct format.
      
      TODO: Improve this process, perhaps using the YAML library more extensiviely
      """
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
                  cv2.imwrite(filename, img)
                  next_time = next_time + interval
      
  def _print_bag_report(self, bagconfig):
      total_xml_count = 0
      total_img_count = 0
      for segment in bagconfig['segments']:
          print "\t\tGenerating Report for segment '{}' with topic '{}'".format(segment['name'], segment['topic'])
          xml_path = os.path.join(self.labelme_dir, 'Annotations', segment['name'], segment['topic'].replace('/', '@'))
          img_path = os.path.join(self.labelme_dir, 'Images', segment['name'], segment['topic'].replace('/', '@'))
          xml_count = 0
          img_count = 0
          for xmlfile in os.listdir(xml_path):
              xml_count += 1
          for imgfile in os.listdir(img_path):
              img_count += 1
          print "\t\t\t{}/{} images labeled in this segment ({}%)".format(xml_count, img_count, round(100.0*xml_count/img_count,2))
          total_xml_count += xml_count
          total_img_count += img_count
      print "\t\t{}/{} images labeled in this bag ({}%)".format(total_xml_count, total_img_count, round(100.0*total_xml_count/total_img_count,2))
      return total_xml_count, total_img_count

  def _parse_label_xml(self, filename):
      msg = LabeledObjects()
      e = xml.etree.ElementTree.parse(filename).getroot()
      for obj in e.findall('object'):
          obj_msg = LabeledObject()
          obj_msg.name = obj.find('name').text
          obj_msg.id = int(obj.find('id').text)
          parent = obj.find('parts').find('ispartof').text
          if parent != None:
            obj_msg.parent_id = int(parent)
          attributes = obj.find('attributes').text
          if attributes != None:
              obj_msg.attributes = attributes
          polygon = obj.find('polygon')
          for pt in polygon.findall('pt'):
            x = int(pt.find('x').text)
            y = int(pt.find('y').text)
            obj_msg.polygon.append(Point(x, y, 0))
          msg.objects.append(obj_msg)
      return msg

  def _get_labels(self, bag_config):
      labels = {}
      for segment in bag_config['segments']:
          labels[segment['topic']] = {}
      for segment in bag_config['segments']:
          path = os.path.join(self.labelme_dir, 'Annotations', segment['name'], segment['topic'].replace('/', '@'))
          print "\t\tExtracting LabelMe annotations from {}".format(path)
          for xmlfile in os.listdir(path):
              stamp = xmlfile.split('.xml', 1)
              assert len(stamp) == 2
              labels[segment['topic']][stamp[0]] = os.path.join(path, xmlfile)
      return labels

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
                  print "\t\tFound matching label at {} {}".format(topic, str(msg.header.stamp))
                  out.write(topic+'/labels', labels, t+rospy.Duration(0.0000001))
        out.write(topic, msg, t)
      out.flush()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Generates rosbags based on LabelMe data and visa/versa')
    parser.add_argument('config', type=str,
                        help='YAML file specifying what bags to read and extract images from. See example YAML for details')
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
