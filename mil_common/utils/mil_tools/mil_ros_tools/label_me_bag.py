#!/usr/bin/env python

"""
Converts bags to labelme images, or labelme annotations to bags.

Uses a YAML file to specify what times and topics in a set of bag files
will be used to extract images for labeling. These images will be placed
in the correct folder to be displayed in the LabelMe web interface.

Once images are labeled, run this command again with --inject-annotations
to parse the labeled images and inject them as a ros message into a new bag file.

With the -r flag, a report will be generated based on the specified YAML
about how many images are labeled in LabelMe. If this returns 100%,
all images generated from the bag are now labeled and can be injected
back into a bag.

"""

from __future__ import division
import argparse
import yaml
import xml.etree.ElementTree
import os
import rosbag
import rospy
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point
from mil_msgs.msg import LabeledObjects, LabeledObject


class BagConfig(object):
    '''
    Stores the configuration for one bag to label.
    '''
    @classmethod
    def default_name(cls, filename):
        '''
        Default name is just filename stripped of any directories
        or extensions.
        '''
        return os.path.splitext(os.path.split(filename)[1])

    def __init__(self, config):
        if 'file' not in config:
            raise Exception('Config for bag has no filename')
        self.filename = config['file']
        if 'topics' not in config:
            raise Exception('{} config has no topics listed'.format(self.filename))
        self.topics = config['topics']
        if not isinstance(self.topics, list):
            self.topics = [self.topics]
        self.start = config['start'] if 'start' in config else None
        self.stop = config['stop'] if 'stop' in config else None
        self.freq = config['freq'] if 'freq' in config else None
        self.name = config['name'] if 'name' in config else self.default_name(self.filename)
        self.outfile = config['outfile'] if 'outfile' in config else self.filename


class BagToLabelMe(object):
    """
    Interfaces between ROS bags and LabelMe images.

    Can be used to insert images from ROS bags into LabelMe instalation
    or extract LabelMe annotations into a ROS bag

    TODO:
    - add enocoding option somewhere for how bag image will be encoded to a jpg (rgb, mono, etc)
    """

    def __init__(self, config, labelme_dir='labelme', verbose=False, indir='', outdir='', force=False):
        """
        @param config: configuration dictionary in valid format, also see from_yaml_file
        @param labelme_dir: directory of labelme instance
        @param verbose: if true, print more about what the program is doing
        @param indir: directory input bags are in
        @param outdir: directory to put
        @param force: If True, will override exsisting bag files when extracting labels
        """
        self.bags = []
        if 'bags' not in config:
            raise Exception('No bags list in config')
        for i, bag in enumerate(config['bags']):
            self.bags.append(BagConfig(bag))
        self.verbose = verbose
        self.bridge = CvBridge()
        self.labelme_dir = labelme_dir
        self.indir = indir
        self.outdir = outdir
        self.force = force

    @classmethod
    def from_yaml_file(cls, filename, **kwargs):
        '''
        Contructs a BagToLabelMe object from a specified YAML file. Simply loads
        the yaml file and forwards the rest to the contructor.
        '''
        f = open(filename, 'r')
        config = yaml.load(f)
        return cls(config, **kwargs)

    def _print(self, string, *args):
        if self.verbose:
            print string.format(*args)

    def _name_encode(self, string):
        '''
        Returns string, with / and other invalid characters
        for labelme replaced.
        '''
        return string.replace('/', '@')

    def _images_directory(self, name, topic):
        '''
        Returns the path where labelme keeps images given
        the bag config name and a topic
        '''
        return os.path.join(self.labelme_dir, 'Images',
                            name,
                            self._name_encode(topic))

    def _annotations_directory(self, name, topic):
        '''
        Returns the path where lableme keeps labels given
        the bag config name and a topic
        '''
        return os.path.join(self.labelme_dir, 'Annotations',
                            name,
                            self._name_encode(topic))

    def read_bags(self):
        """
        Crawls through all bags in config YAML, placing images from the specified
        topics and the specified frequency into corresponding folders within
        Images of the labelme instalation.

        These folders will be created as follows if not already:
        LABELME_DIR/Images/SEGEMENT_NAME/TOPIC_NAME
        where TOPIC_NAME replaces all / with @ for compatibility with filesystems.
        """
        for bag in self.bags:
            self._read_bag(bag)

    def _read_bag(self, config):
        """
        Internal. Gets images from a single bag for labeling for a single bag
        """
        filename = os.path.join(self.indir, config.filename)
        bag = rosbag.Bag(filename)
        self._print("\tGetting images from '{}'", config.name)

        # Get path in labelme to put images by topic in this bag config
        paths = {}
        for t in config.topics:
            path = self._images_directory(config.name, t)
            if not os.path.exists(path):
                os.makedirs(path)
            paths[t] = path

        # Load start, stop, and frequency from config or defaults
        _, _, first_time = bag.read_messages().next()
        start = first_time + rospy.Duration(config.start) if config.start else first_time
        stop = first_time + rospy.Duration(config.stop) if config.stop else None
        interval = rospy.Duration(1.0 / config.freq) if config.freq else rospy.Duration(0)

        # Crawl through bag in configured time and frequency, writing images into labelme
        next_time = start + interval
        for topic, msg, time in bag.read_messages(topics=config.topics, start_time=start, end_time=stop):
            if time >= next_time:
                img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
                filename = os.path.join(paths[topic], str(msg.header.stamp) + '.jpg')
                cv2.imwrite(filename, img)
                next_time = time + interval

    def extract_labels(self):
        """
        Finds labeled XML files in LabelMe directories determined from the
        config file, placing parsed annotation data as ros messages in a new bag
        """
        self._print("Extracting labels for all bags in config")
        for bag in self.bags:
            self._extract_labels_bag(bag)

    def _extract_labels_bag(self, bag):
        '''
        Internal. Extracts labels for one bag config.
        '''
        # Open output bag to write labels (along with original content) to
        outfilename = os.path.join(self.outdir, bag.outfile)
        if os.path.exists(outfilename):  # If output bag already exists, only override if force glag is set
            if self.force:
                self._print('{} already exists. OVERRIDING'.format(outfilename))
            else:
                self._print('{} already exists. Not overriding'.format(outfilename))
                return
        infilename = os.path.join(self.indir, bag.filename)
        inbag = rosbag.Bag(infilename)
        outbag = rosbag.Bag(outfilename, mode='w')

        # Generate dictionary of topics to dictionaries of stamps to
        # ex: labels['/camera'][12756153011397] -> rosmsg of labels at this time
        labels = {}
        for topic in bag.topics:
            path = self._annotations_directory(bag.name, topic)
            if not os.path.isdir(path):
                continue
            labels[topic] = {}
            for filename in os.listdir(path):
                msg = self.label_to_msg(os.path.join(path, filename))
                stamp = os.path.splitext(filename)[0]
                labels[topic][stamp] = msg

        # Go through bag, on any matched images and labels, write labels
        for topic, msg, t in inbag.read_messages(topics=labels.keys()):
            if msg._type == 'sensor_msgs/Image' and str(msg.header.stamp) in labels[topic]:
                label = labels[topic][str(msg.header.stamp)]
                label.header = msg.header
                outbag.write(topic + '/labels', label, t)
            outbag.write(topic, msg, t)
        outbag.close()

    def completion_report(self):
        """
        Compares how many images are present in each LabelMe directory affected
        by the config YAML to the number of annotated XML files produced by
        LabelMe in each of these directories.

        Prints out these counts in percentages by segement, bag, and overall
        """
        self._print("Generating completion report for all bags in config")
        total_xml_count = 0
        total_img_count = 0
        for bag in self.bags:
            x, i = self._completion_bag(bag)
            total_xml_count += x
            total_img_count += i
        if total_img_count == 0:
            print "{}/{} TOTAL images labeled (0%)".format(total_xml_count, total_img_count)
        else:
            print "{}/{} TOTAL images labeled ({:.1%})".format(total_xml_count, total_img_count,
                                                               total_xml_count / total_img_count)

    def _completion_bag(self, bag):
        """
        Compares number of labeled XML files in LabelMe for a given
        bag specified in YAML config to number of images in LabelMe for this
        bag.

        If verbose is enabled, prints info for each bag. Otherwise,
        just returns total counts.

        Called by print_report for each bag in config.
        """
        xml_count = 0
        img_count = 0
        for t in bag.topics:
            xml_path = self._annotations_directory(bag.name, t)
            img_path = self._images_directory(bag.name, t)
            if os.path.isdir(xml_path):
                for xmlfile in os.listdir(xml_path):
                    xml_count += 1
            if os.path.isdir(img_path):
                for imgfile in os.listdir(img_path):
                    img_count += 1
        if img_count == 0:
            self._print("\t{}/{} images labeled in {} (0%)", xml_count, img_count, bag.name)
        else:
            self._print("\t{}/{} images labeled in {} ({:.1%})", xml_count, img_count, bag.name,
                        xml_count / img_count)
        return xml_count, img_count

    @staticmethod
    def label_to_msg(filename):
        """
        Given an XML file containing annotation data in LabelMe format,
        generates and returns a mil_msgs/LabeledObjects message
        containing the same data.
        """
        msg = LabeledObjects()
        e = xml.etree.ElementTree.parse(filename).getroot()
        for obj in e.findall('object'):
            obj_msg = LabeledObject()
            obj_msg.name = obj.find('name').text
            obj_msg.id = int(obj.find('id').text)
            parent = obj.find('parts').find('ispartof').text
            if parent is not None:
                obj_msg.parent_id = int(parent)
            attributes = obj.find('attributes').text
            if attributes is not None:
                obj_msg.attributes = attributes
            polygon = obj.find('polygon')
            for pt in polygon.findall('pt'):
                x = int(pt.find('x').text)
                y = int(pt.find('y').text)
                obj_msg.polygon.append(Point(x, y, 0))
            msg.objects.append(obj_msg)
        return msg


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generates rosbags based on LabelMe data and visa/versa')
    parser.add_argument('config', type=str,
                        help='YAML file specifying what bags to read and extract images from.\
                              See example YAML for details')
    parser.add_argument('--labelme-dir', '-d', dest='dir', type=str, default="",
                        help='root directory of labelme instalation. \nDefaults to current directory.')
    parser.add_argument('--bag-dir', '-b', dest="bag_dir", type=str, default="",
                        help="directory to resolve relative paths specifed in YAML for input bags. \n\
                              Defaults to current directory.")
    parser.add_argument('--output-dir', '-o', dest="output_dir", type=str, default="",
                        help="directory to resolve relative paths specified in YAML for output (labeled) bags. \n\
                              Defaults to current directory.")
    parser.add_argument('--extract', '-e', dest='extract_labels', action='store_true',
                        help='Instead of putting bag images into LabelMe, read annotations from labelme,\
                              inserting them into a new bag as specified in config')
    parser.add_argument('--generate-report', '-r', dest='do_report', action='store_true',
                        help='Read annotations from labelme and produces a report on labeling coverage')
    parser.add_argument('--dry-run', '-n', dest='do_dry_run', action='store_true',
                        help='No op, just verify parsed config of yaml')
    parser.add_argument('--verbose', '-v', dest='verbose', action='store_true',
                        help='Print extra information about what the script is doing')
    parser.add_argument('--force', '-f', dest='force', action='store_true',
                        help='Override bags if they already exist when running extract mode')
    args = parser.parse_args()
    bag_to_labelme = BagToLabelMe.from_yaml_file(args.config, labelme_dir=args.dir, verbose=args.verbose,
                                                 indir=args.bag_dir, outdir=args.output_dir, force=args.force)
    if args.do_dry_run:
        pass
    elif args.extract_labels:
        bag_to_labelme.extract_labels()
    elif args.do_report:
        bag_to_labelme.completion_report()
    else:
        bag_to_labelme.read_bags()
