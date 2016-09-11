#!/usr/bin/python
from txros import util, NodeHandle
from twisted.internet import defer, reactor
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError

from navigator_tools import image_helpers
import numpy as np
import argcomplete
import sys
import argparse
import cv2


class Trackbars(object):
    '''
    Keeps track of the different colorspace trackbars
    '''
    def __init__(self, thresh_type):
        self.thresh_type = list(thresh_type)
        [cv2.createTrackbar(name + '_high', 'parameters', 0, 255 if name != 'h' else 179, lambda a: a) for name in self.thresh_type]
        [cv2.createTrackbar(name + '_low', 'parameters', 0, 255 if name != 'h' else 179, lambda a: a) for name in self.thresh_type]

        cv2.waitKey(10)

    def get_bounds(self):
        upper_bounds = np.array([cv2.getTrackbarPos(name + '_high', 'parameters') for name in self.thresh_type])
        lower_bounds = np.array([cv2.getTrackbarPos(name + '_low', 'parameters') for name in self.thresh_type])
        return lower_bounds, upper_bounds


class Thresholder(object):
    '''
    Does the thresholding and manages the windows associated with that
    '''
    def __init__(self, img, thresh_type='bgr'):
        cv2.namedWindow('parameters', cv2.WINDOW_NORMAL)
        cv2.namedWindow('mask')
        cv2.imshow('mask', np.zeros_like(img[:,:,0]))

        self.image = img

        self.thresh_type = thresh_type
        self.trackbars = Trackbars(thresh_type)

        self.lower = None
        self.upper = None

    def update_mask(self):
        self.lower, self.upper = self.trackbars.get_bounds()
        self.mask = cv2.inRange(self.image if self.thresh_type == 'bgr' else cv2.cvtColor(self.image, cv2.COLOR_BGR2HSV),
                                self.lower, self.upper)
        cv2.imshow("mask", self.mask)

    def to_dict(self):
        lower = map(float, self.lower)
        upper = map(float, self.upper)
        return {'color_space':self.thresh_type, 'ranges': {'lower': lower, 'upper': upper}, 'invert': False}


@util.cancellableInlineCallbacks
def main(param_prefix, args):
    nh = yield NodeHandle.from_argv("on_the_fly_mission_runner", anonymous=True)

    image_sub = yield nh.subscribe(args.topic_name, Image)
    img = yield util.wrap_timeout(image_sub.get_next_message(), 5).addErrback(errback)

    np_img = image_helpers.get_image_msg(img, "bgr8")
    cv2.imshow(args.topic_name, np_img)
    t = Thresholder(np_img, 'hsv' if args.hsv else 'bgr')

    k = 0
    while k != ord('q'):  # q to quit without saving
        t.update_mask()
        k = cv2.waitKey(50) & 0xFF

        if k == ord('s'):  # s to save parameters
            print "Saving params:"
            temp = t.to_dict()
            print temp
            nh.set_param(param_prefix, temp)
            break

    cv2.destroyAllWindows()
    reactor.stop()


def errback(e):
    '''Just for catching errors'''
    print "Error when looking for image."
    print " - You probably entered the wrong image topic."
    reactor.stop()


def do_parsing():
    usage_msg = "Useful for doing on-the-fly color thresholding."
    desc_msg = "Pass the name of the topic to listen to and the parameter family to set."

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='topic_name',
                        help="The topic name you'd like to listen to.")
    parser.add_argument(dest='parameter_family',
                        help="This script will set the rosparams: \n\t `parameter_family`/bgr_high \n\t `parameter_family`/bgr_low \n OR \
                              \n\t `parameter_family`/hsv_high \n\t `parameter_family`/hsv_high \n (depending on --hsv)")
    parser.add_argument('--hsv', action='store_true',
                        help="Use HSV instead of BGR")

    args = parser.parse_args(sys.argv[1:])
    print 'Using {}'.format('HSV' if args.hsv else 'BGR')

    return args.parameter_family, args


if __name__ == '__main__':
    param_prefix, args = do_parsing()
    reactor.callWhenRunning(main, param_prefix, args)
    reactor.run()
