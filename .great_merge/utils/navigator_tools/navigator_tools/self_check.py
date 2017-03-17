#!/usr/bin/env python
from twisted.internet import defer, reactor
import traceback
import signal
import txros


class FancyPrint(object):
    GOOD = '\033[32m'
    BAD = '\033[31m'
    NORMAL = '\033[0m'
    BOLD = '\033[1m'

    @classmethod
    def okay(self, text):
        print self.GOOD + text + self.NORMAL

    @classmethod
    def error(self, text):
        print self.BAD + text + self.NORMAL

from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
def add_camera_feeds(nh, cam_name, image_type="image_raw"):
    ''' Returns subscribers to the raw and compressed `image_type` '''

    raw = '/{}/{}'.format(cam_name, image_type)
    compressed = '/{}/{}/compressed'.format(cam_name, image_type)
    return nh.subscribe(raw, Image).get_next_message(), \
           nh.subscribe(compressed, CompressedImage).get_next_message()

@txros.util.cancellableInlineCallbacks
def main():
    nh = yield txros.NodeHandle.from_argv("self_checker")
    # Add deferreds to this dict to be yieleded on and checked later
    topics = {}

    # General Subs
    from nav_msgs.msg import Odometry
    topics['odom'] = nh.subscribe('odom', Odometry).get_next_message()

    from sensor_msgs.msg import Joy
    topics['joy'] = nh.subscribe('joy', Joy).get_next_message()

    # Perception Subs
    topics['right_images'], topics['right_compressed'] = add_camera_feeds(nh, "right")
    topics['stereo_right_images'], topics['stereo_right_compressed'] = add_camera_feeds(nh, "stereo/right")
    topics['stereo_left_images'], topics['stereo_left_compressed'] = add_camera_feeds(nh, "stereo/left")

    from sensor_msgs.msg import PointCloud2
    topics['velodyne'] = nh.subscribe('/velodyne_points', PointCloud2).get_next_message()

    # Thrusters
    from roboteq_msgs.msg import Feedback
    topics['BL_motor'] = nh.subscribe('/BL_motor/Feedback', Feedback).get_next_message()
    topics['BR_motor'] = nh.subscribe('/BR_motor/Feedback', Feedback).get_next_message()
    topics['FL_motor'] = nh.subscribe('/FL_motor/Feedback', Feedback).get_next_message()
    topics['FR_motor'] = nh.subscribe('/FR_motor/Feedback', Feedback).get_next_message()

    for name, sub in topics.iteritems():
        try:
            # Bold the name so it's distinct
            fancy_name = FancyPrint.BOLD + name + FancyPrint.NORMAL
            print " - - - - Testing for {}".format(fancy_name)

            result = yield txros.util.wrap_timeout(sub, 2)  # 2 second timeout should be good
            if result is None:
                FancyPrint.error("[ FAIL ] Response was None from {}".format(fancy_name))
            else:
                FancyPrint.okay("[ PASS ] Response found from {}".format(fancy_name))

        except:
            FancyPrint.error("[ FAIL ] No response from {}".format(fancy_name))


if __name__ == '__main__':
    txros.util.launch_main(main)