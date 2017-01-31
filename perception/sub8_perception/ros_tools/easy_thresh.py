#!/usr/bin/python
# PYTHON_ARGCOMPLETE_OK

import argcomplete
import sys
import argparse
import rospy

all_topics = rospy.get_published_topics()
topics = [topic[0] for topic in all_topics if topic[1] == 'sensor_msgs/Image']

usage_msg = ("Name an image topic, we'll subscribe to it and grab the first image we can. " +
             "Then click a rectangle around the area of interest")
desc_msg = "A tool for making threshold determination fun!"

parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
parser.add_argument(dest='topic_name',
                    help="The topic name you'd like to listen to",
                    choices=topics)
parser.add_argument('--hsv', action='store_true',
                    help="Would you like to look at hsv instead of bgr?"
                    )

argcomplete.autocomplete(parser)
args = parser.parse_args(sys.argv[1:])
if args.hsv:
    print 'Using HSV instead of bgr'
prefix = 'hsv' if args.hsv else 'bgr'

# Importing these late so that argcomplete can run quickly
from sub8_vision_tools import visual_threshold_tools  # noqa
from sub8_ros_tools.image_helpers import Image_Subscriber  # noqa
import cv2  # noqa
import numpy as np  # noqa
import os  # noqa
from sklearn import cluster  # noqa
os.system("export ETS_TOOLKIT=qt4")


class Segmenter(object):
    def __init__(self):
        self.is_done = False
        self.corners = []
        self.state = 0

    def mouse_cb(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.is_done:
                print 'click'
                self.corners.append(np.array([x, y]))
                self.state += 1
                if self.state >= 4:
                    print 'done'
                    self.is_done = True
                    self.state = 0

        if event == cv2.EVENT_RBUTTONDOWN:
            pass

    def segment(self):
        while(not self.is_done and not rospy.is_shutdown()):
            if cv2.waitKey(50) & 0xFF == ord('q'):
                break

        self.is_done = False
        rect = cv2.minAreaRect(np.array([np.array(self.corners)]))
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        return box


class ImageGetter(object):
    def __init__(self, topic_name='/down/left/image_raw'):
        self.sub = Image_Subscriber(topic_name, self.get_image)

        print 'getting topic', topic_name
        self.frame = None
        self.done = False

    def get_image(self, msg):
        self.frame = msg
        self.done = True

    def wait_for_image(self):
        while not self.done and not rospy.is_shutdown():
            if cv2.waitKey(50) & 0xFF == ord('q'):
                exit()


if __name__ == '__main__':
    rospy.init_node('easy_thresh')

    # Do the import after arg parse

    ig = ImageGetter(args.topic_name)
    ig.wait_for_image()
    print 'Got image'
    frame_initial = np.copy(ig.frame)

    cv2.namedWindow("color")
    seg = Segmenter()
    cv2.setMouseCallback("color", seg.mouse_cb)

    frame_unblurred = frame_initial[::2, ::2, :]
    frame = frame_unblurred

    if args.hsv:
        analysis_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    else:
        analysis_image = frame

    draw_image = np.copy(frame_unblurred)
    seg_image = np.zeros_like(frame_unblurred[:, :, 0])

    cv2.imshow("color", frame_unblurred)

    box = seg.segment()
    print 'finished'

    cv2.drawContours(seg_image, [box], 0, 1, -2)

    hsv_in_box = analysis_image[seg_image.astype(np.bool)]
    hsv_list = np.reshape(hsv_in_box, (-1, 3))

    clust = cluster.KMeans(n_clusters=2)
    print 'done clustering'

    clust.fit(hsv_list)

    hsv_dsamp = hsv_list
    labels_dsamp = clust.labels_

    visual_threshold_tools.points_with_labels(
        hsv_dsamp[:, 0],
        hsv_dsamp[:, 1],
        hsv_dsamp[:, 2],
        labels_dsamp,
        scale_factor=5.0,
    )

    thresholder = visual_threshold_tools.make_extent_dialog(
        ranges=visual_threshold_tools.color_ranges[prefix],
        image=analysis_image
    )

    while not rospy.is_shutdown():
        if cv2.waitKey(50) & 0xFF == ord('q'):
            break

    ranges = thresholder.ranges
    labels = visual_threshold_tools.np_inrange(hsv_dsamp, ranges[:, 0], ranges[:, 1])

    # Print out thresholds that can be put in the configuration yaml
    low = ranges[:, 0]
    print '  {}_low: [{}, {}, {}]'.format(prefix, low[0], low[1], low[2])

    high = ranges[:, 1]
    print '  {}_high: [{}, {}, {}]'.format(prefix, high[0], high[1], high[2])

    print 'np.' + repr(ranges)

    cv2.destroyAllWindows()
