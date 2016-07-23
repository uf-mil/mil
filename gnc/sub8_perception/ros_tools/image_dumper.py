#!/usr/bin/python

import os
import re
import sys
import fnmatch
import argparse
from tqdm import tqdm

import cv2
import rospy
import roslib
import rosbag

from cv_bridge import CvBridge, CvBridgeError

# encoding=utf8
reload(sys)
sys.setdefaultencoding('utf8')


class ImageHandler:
    def __init__(self, filepath, savepath, bag):
        self.bag = bag
        self.msgs = 0   # Used to show remaining frames
        self.image_index = 0
        self.image_topics = []
        self.bridge = CvBridge()
        self.bagname = filepath.split('/')[-1]  # Returns ../../bagname.bag

        if savepath is not None:
            # Alternative save path (i.e. want to save to ext. drive)
            if savepath[-1] != '/':
                savepath += '/'
            self.working_dir = savepath + self.bagname.split('.')[0]\
                                        + '_images'
        else:
            # Save path defaults to launch path
            self.working_dir = filepath.split('.')[0] + '_images'

        if not os.path.exists(self.working_dir):
            os.makedirs(self.working_dir)

        print '\033[1m' + self.bagname + ':\033[0m'
        print 'Saving images to: ', self.working_dir

        self.parse_bag()
        self.save_images()

    def parse_bag(self):
        types = []
        bag_msg_cnt = []
        topic_status = False
        topics = bag.get_type_and_topic_info()[1].keys()

        for i in range(0, len(bag.get_type_and_topic_info()[1].values())):
            types.append(bag.get_type_and_topic_info()[1].values()[i][0])
            bag_msg_cnt.append(bag.get_type_and_topic_info()[1].values()[i][1])

        topics = zip(topics, types, bag_msg_cnt)

        for topic, type, count in topics:
            # Want to ignore image topics other than /image_raw
            # TODO: Make this changeable
            match = re.search(r'\mono|rect|theora|color\b', topic)
            if match:
                pass
            elif type == 'sensor_msgs/Image':
                if topic_status is False:
                    print 'Topic(s):'
                    topic_status = True
                print '   ' + topic
                self.image_topics.append(topic)
                self.msgs = self.msgs + count

    def save_images(self):
        # TODO: Add ability to pickup where it last left off
        with tqdm(total=self.msgs) as pbar:
            for topic, msg, t in self.bag.read_messages(
                                            topics=self.image_topics):
                try:
                    cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                    cv2.imwrite(self.working_dir + '/' +
                                str(self.image_index) + '.png', cv_image)
                    self.image_index = self.image_index + 1
                except CvBridgeError, e:
                    print e
                pbar.update(1)
            self.bag.close()
        print self.image_index + 1, 'images saved to', self.working_dir, '\n'

if __name__ == '__main__':

    desc_msg = ('Automated tool that traverses a directory dumping all of' +
                'the images from all non-rectificed, non-mono, and non-' +
                'theora topics found in all of the ROS bag files encountered.')

    parser = argparse.ArgumentParser(description=desc_msg)

    parser.add_argument('-f', '--filepath',
                        help='File path containing the ROS bags')
    parser.add_argument('-s', '--savepath',
                        help='Path where the images will be saved')
    parser.add_argument('-r', '--resize',
                        help='Resolution to resize images to')

    args = parser.parse_args()

    # TODO: Path validation can be moved to ImageHandler class
    if args.filepath is not None:
        filepath = args.filepath
    else:
        print 'No bag source provided'
        filepath = sys.path[0]

    if args.savepath is not None:
        savepath = args.savepath
        print 'Setting save path to:', savepath
    else:
        savepath = None

    if args.resize:
        # We're not ready for the future yet
        pass

    print '\033[1mFilepath:', filepath + '\033[0m'  # Print working directory

    matches = []
    bag_count = 0

    if filepath[-4:] == '.bag':
        bag = rosbag.Bag(filepath)
        ImageHandler(filepath, savepath, bag)
    else:

        # Generate list of file paths containing bag files
        for root, dirnames, filenames in os.walk(filepath):
            for filename in fnmatch.filter(filenames, '*.bag'):
                if not filename.startswith('.'):
                    matches.append(os.path.join(root, filename))

        print '\033[1m' + str(len(matches)) + ' bags found\033[0m\n'

        # Iterate through found bags
        for bag_dir in matches:
            bag = rosbag.Bag(bag_dir)
            try:
                ImageHandler(bag_dir, savepath, bag)
                bag_count = bag_count + 1
            except rospy.ROSInterruptException:
                pass

        print 'Processed', bag_count, 'bags.\n'

    print "Done!"
