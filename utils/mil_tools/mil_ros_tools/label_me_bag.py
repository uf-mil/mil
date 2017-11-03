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


class BagToLabelMe():
    """
    Interfaces between ROS bags and LabelMe images.

    Can be used to insert images from ROS bags into LabelMe instalation
    or extract LabelMe annotations into a ROS bag

    TODO:
    - add enocoding option somewhere for how bag image will be encoded to a jpg (rgb, mono, etc)
    """

    def __init__(self, config, labelme_dir, verbose=False, indir='', outdir=''):
        """
        Generates class that can be used to insert or extract images to/from LabelMe

        config: path to YAML file following format of the example yaml file in this package
        labelme_dir: root directory of labelme instalation (should contain Images and Annotations directories)
        """
        self.verbose = verbose
        if not os.path.isdir(labelme_dir):
            raise Exception(
                "Labelme directory {} does not exsist".format(labelme_dir))

        config_file = open(config, 'r')
        self.config = yaml.load(config_file)
        self._verify_yaml()
        self.bridge = CvBridge()
        self.labelme_dir = labelme_dir
        self.indir = indir
        self.outdir = outdir

    def _print(self, string, *args):
        if self.verbose:
            print string.format(*args)

    def _verify_yaml(self):
        """
        Called by constructor to ensure YAML follows correct format.

        TODO: Improve this process, perhaps using the YAML library more extensiviely
        """
        def verify_attr(obj, attr, objname, throw=True):
            if not throw:
                return attr in obj
            if attr not in obj:
                raise Exception(
                    "{} does not have required attribute {} in config".format(objname, attr))
        verify_attr(self.config, 'bags', 'Config')
        self._print('Pulling segments from {} bag(s):',
                    len(self.config['bags']))
        for i, bag in enumerate(self.config['bags']):
            verify_attr(bag, 'file', 'Bag')
            if 'file' not in bag:
                raise Exception(
                    "Bag does not contain 'file' attribute {}".format(bag))
            verify_attr(bag, 'segments', bag['file'])
            if 'combined' not in bag:
                self.config['bags'][i]['combined'] = False
            self._print('\tFound {} segments for {}:',
                        len(bag['segments']), bag['file'])
            for j, segment in enumerate(bag['segments']):
                verify_attr(segment, 'topics', 'segment')
                # Ensure topics is still a list
                if type(segment['topics']) == str:
                    segment['topics'] = [segment['topics']]
                verify_attr(segment, 'name', 'segment')
                self._print("\t\tFound segment '{}' of topics '{}' from {} to {} every {}",
                            segment['name'], segment['topics'],
                            segment['start'] if 'start' in segment else 'start',
                            segment['stop'] if 'stop' in segment else 'stop',
                            str(segment['freq']) + ' seconds' if 'freq' in segment else 'frame')

    def read_bags(self):
        """
        Crawls through all bags in config YAML, placing images from the specified
        topics and the specified frequency into corresponding folders within
        Images of the labelme instalation.

        These folders will be created as follows if not already:
        LABELME_DIR/Images/SEGEMENT_NAME/TOPIC_NAME
        where TOPIC_NAME replaces all / with @ for compatibility with filesystems.
        """
        for bag in self.config['bags']:
            self._print("Opening {}", bag['file'])
            self._read_bag(bag['file'], bag['segments'])

    def extract_labels(self):
        """
        Finds labeled XML files in LabelMe directories determined from the
        config file, placing parsed annotation data as ros messages in a new bag
        """
        self._print("Extracting labels for all bags in config")
        for bag in self.config['bags']:
            self._print(
                "\tExtracting LabelMe annotations for {}".format(bag['file']))
            self._extract_labels_bag(bag)

    def print_report(self):
        """
        Compares how many images are present in each LabelMe directory affected
        by the config YAML to the number of annotated XML files produced by
        LabelMe in each of these directories.

        Prints out these counts in percentages by segement, bag, and overall
        """
        self._print("Generating completion report for all bags in config")
        total_xml_count = 0
        total_img_count = 0
        for bag in self.config['bags']:
            self._print("\tGenerating completion report for {}", bag['file'])
            x, i = self._print_bag_report(bag)
            total_xml_count += x
            total_img_count += i
        if total_img_count == 0:
            print "{}/{} TOTAL images labeled (0%)".format(total_xml_count, total_img_count)
        else:
            print "{}/{} TOTAL images labeled ({:.1%})".format(total_xml_count, total_img_count,
                                                               total_xml_count / total_img_count)

    def _name_encode(self, string):
        '''
        Used to convert topic and
        '''
        return string.replace('/', '@')

    def _read_bag(self, bagfile, segments):
        """
        Crawls through a single bag specified in the config YAML, placing
        images into the Images/ directory of labelme.

        Called me read_bags for each bag in YAML
        """
        bag = self._get_input_bag(bagfile)
        _, _, first_time = bag.read_messages().next()
        for segment in segments:
            self._print("\tProccessing Segment '{}'", segment['name'])
            # Put images in LABELMEDIR/Images/SEGMENTNAME/TOPIC, where TOPIC
            # replaces / with @
            paths = {}
            for t in segment['topics']:
                path = os.path.join(self.labelme_dir, 'Images',
                                    self._name_encode(segment['name']),
                                    self._name_encode(t))
                if not os.path.exists(path):
                    os.makedirs(path)
                paths[t] = path
            # Ensure start and stop are passed as None to read_messages if not
            # defined
            start = None
            stop = None
            interval = rospy.Duration(0)
            if 'start' in segment:
                start = first_time + rospy.Duration(segment['start'])
            if 'stop' in segment:
                stop = first_time + rospy.Duration(segment['stop'])
            if 'freq' in segment:
                interval = rospy.Duration(1.0 / segment['freq'])
            next_time = start if start is not None else rospy.Time(0)
            # Crawl through bag between start and stop time at freq
            for topic, msg, time in bag.read_messages(topics=segment['topics'], start_time=start, end_time=stop):
                if time >= next_time:
                    # If enough time has elapsed, put an image in this segment's directory,
                    # named by the frame's timestamp converted to a string:
                    # str(ros.Time(stamp))
                    img = self.bridge.imgmsg_to_cv2(
                        msg, desired_encoding='rgb8')
                    filename = os.path.join(
                        paths[topic], str(msg.header.stamp) + '.jpg')
                    cv2.imwrite(filename, img)
                    next_time = time + interval

    def _print_bag_report(self, bagconfig):
        """
        Compares number of labeled XML files in LabelMe for a given
        bag specified in YAML config to number of images in LabelMe for this
        bag.

        If verbose is enabled, prints info for each segment. Otherwise,
        just returns total counts.

        Called by print_report for each bag in config.
        """
        total_xml_count = 0
        total_img_count = 0
        for segment in bagconfig['segments']:
            self._print("\t\tGenerating Report for segment '{}'",
                        segment['name'])
            xml_count = 0
            img_count = 0
            for t in segment['topics']:
                xml_path = os.path.join(self.labelme_dir, 'Annotations',
                                        self._name_encode(segment['name']),
                                        self._name_encode(t))
                img_path = os.path.join(self.labelme_dir, 'Images',
                                        self._name_encode(segment['name']),
                                        self._name_encode(t))
                if os.path.isdir(xml_path):
                    for xmlfile in os.listdir(xml_path):
                        xml_count += 1
                if os.path.isdir(img_path):
                    for imgfile in os.listdir(img_path):
                        img_count += 1
            if img_count == 0:
                self._print(
                    "\t\t\t{}/{} images labeled in this segment (0%)", xml_count, img_count)
            else:
                self._print("\t\t\t{}/{} images labeled in this segment ({:.1%})", xml_count, img_count,
                            xml_count / img_count)
            total_xml_count += xml_count
            total_img_count += img_count
        if total_img_count == 0:
            self._print("\t\t{}/{} images labeled in this bag (0%)",
                        total_xml_count, total_img_count)
        else:
            self._print("\t\t{}/{} images labeled in this bag ({:.1%})", total_xml_count, total_img_count,
                        total_xml_count / total_img_count)
        return total_xml_count, total_img_count

    def _parse_label_xml(self, filename):
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

    def _get_labels_segment(self, segment, labels={}):
        for t in segment['topics']:
            if t not in labels:
                labels[t] = {}
            path = os.path.join(self.labelme_dir, 'Annotations',
                                self._name_encode(segment['name']),
                                self._name_encode(t))
            if not os.path.isdir(path):
                continue
            for xmlfile in os.listdir(path):
                stamp = xmlfile.split('.xml', 1)
                assert len(stamp) == 2
                labels[t][stamp[0]] = os.path.join(path, xmlfile)
        return labels

    def _get_labels(self, bag_config):
        """
        Given a config for a single bag, finds all XML files in LabelMe
        containing data for the segments in this bag.

        returns a dictionary mapping ros.Time stamps to XML files.
        Used by extract_labels to put labelme data back into bag.
        """
        labels = {}
        for segment in bag_config['segments']:
            for t in segment['topics']:
                labels[t] = {}
        for segment in bag_config['segments']:
            labels = self._get_label_segment(segment, labels)
        return labels

    def _get_bag_filename(self, filename, directory):
        base = os.path.splitext(filename)[0]
        assert len(base) != 0
        return os.path.join(directory, base + '.bag')

    def _get_input_bag(self, name):
        filename = self._get_bag_filename(name, self.indir)
        return rosbag.Bag(filename)

    def _get_combined_output_bag(self, bag_config):
        if 'outfile' in bag_config:
            filename = self._get_bag_filename(
                bag_config['outfile'], self.outdir)
            return rosbag.Bag(filename)
        base = os.path.split(bag_config['file'])[1]
        filename = self._get_bag_filename(base, self.outdir)
        directory = os.path.split(filename)[0]
        if not os.path.exists(directory):
            os.makedirs(directory)
        return rosbag.Bag(filename, mode='w')

    def _get_segment_output_bag(self, segment):
        filename = segment['name']
        if 'outfile' in segment:
            filename = segment['outfile']
        filename = self._get_bag_filename(filename, self.outdir)
        directory = os.path.split(filename)[0]
        if not os.path.exists(directory):
            os.makedirs(directory)
        return rosbag.Bag(filename, mode='w')

    def _extract_labels_bag(self, bag_config):
        bag = self._get_input_bag(bag_config['file'])
        if bag_config['combined']:
            label_files = self._get_labels(bag_config)
            out = self._get_combined_output_bag(bag_config)
            for topic, msg, t in bag.read_messages():
                if msg._type == 'sensor_msgs/Image':
                    if topic in label_files:
                        if str(msg.header.stamp) in label_files[topic]:
                            labels = self._parse_label_xml(
                                label_files[topic][str(msg.header.stamp)])
                            labels.header = msg.header
                            out.write(topic + '/labels', labels, t)
                out.write(topic, msg, t)
            out.flush()
            out.close()
        else:
            _, _, first_time = bag.read_messages().next()
            for segment in bag_config['segments']:
                label_files = self._get_labels_segment(segment)
                out = self._get_segment_output_bag(segment)
                start = None
                stop = None
                if 'start' in segment:
                    start = first_time + rospy.Duration(segment['start'])
                if 'stop' in segment:
                    stop = first_time + rospy.Duration(segment['stop'])
                for topic, msg, t in bag.read_messages(start_time=start, end_time=stop):
                    if msg._type == 'sensor_msgs/Image':
                        if topic in label_files:
                            if str(msg.header.stamp) in label_files[topic]:
                                labels = self._parse_label_xml(
                                    label_files[topic][str(msg.header.stamp)])
                                labels.header = msg.header
                                out.write(topic + '/labels', labels, t)
                    out.write(topic, msg, t)
                out.flush()
                out.close()
        bag.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Generates rosbags based on LabelMe data and visa/versa')
    parser.add_argument('config', type=str,
                        help='YAML file specifying what bags to read and extract images from.\
                              See example YAML for details')
    parser.add_argument('--labelme-dir', '-d', dest='dir', type=str, required=True,
                        help='root directory of labelme instalation')
    parser.add_argument('--bag-dir', '-b', dest="bag_dir", type=str, default="",
                        help="directory to resolve relative paths specifed in YAML for input bags")
    parser.add_argument('--output-dir', '-o', dest="output_dir", type=str, default=".",
                        help="directory to resolve relative paths specified in YAML for output (labeled) bags")
    parser.add_argument('--extract', '-e', dest='extract_labels', action='store_true',
                        help='Instead of putting bag images into LabelMe, read annotations from labelme,\
                              inserting them into a new bag as specified in config')
    parser.add_argument('--generate-report', '-r', dest='do_report', action='store_true',
                        help='Read annotations from labelme and produces a report on labeling coverage')
    parser.add_argument('--dry-run', '-n', dest='do_dry_run', action='store_true',
                        help='No op, just verify parsed config of yaml. Use -v for to print info about parsed config.')
    parser.add_argument('--verbose', '-v', dest='verbose', action='store_true',
                        help='Print extra information about what the script is doing')
    args = parser.parse_args()
    bag_to_labelme = BagToLabelMe(args.config, args.dir, verbose=args.verbose,
                                  indir=args.bag_dir, outdir=args.output_dir)
    if args.do_dry_run:
        pass
    elif args.extract_labels:
        bag_to_labelme.extract_labels()
    elif args.do_report:
        bag_to_labelme.print_report()
    else:
        bag_to_labelme.read_bags()
