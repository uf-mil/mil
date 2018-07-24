#!/usr/bin/python
import sys
import cv2
import rospy
import rospkg
import datetime
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from std_srvs.srv import SetBool, SetBoolResponse
from geometry_msgs.msg import Point

rospack = rospkg.RosPack()

# To correctly import utils
sys.path.append(
    rospack.get_path('sub8_perception') + '/ml_classifiers/dice/utils')

from utils import detector_utils  # noqa


class classifier(object):
    def __init__(self):
        rospy.init_node('dice_detection')
        self.bridge = CvBridge()
        self.subscriber = rospy.Subscriber(
            '/camera/front/left/image_rect_color', Image, self.img_callback)
        self.publisher = rospy.Publisher(
            '/vision/dice/debug_rcnn', Image, queue_size=1)

        self.dice_publisher = rospy.Publisher(
            'dice/points', Point, queue_size=1)

        rospy.Service('~enable', SetBool, self.toggle_search)
        self.enabled = False

        # Parameters
        self.num_frames = 0
        self.num_objects_detect = 4
        self.score_thresh = 0.1
        self.im_width = 1920
        self.im_height = 1080
        self.start_time = datetime.datetime.now()

    def toggle_search(self, srv):
        if srv.data:
            self.inference_graph, self.sess = detector_utils.load_inference_graph(
            )
            self.enabled = True
        else:
            self.sess.close()
            self.enabled = False
        return SetBoolResponse(success=True)

    def img_callback(self, data):
        if not self.enabled:
            return
        try:
            print('Working')
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Run image through tensorflow graph
        boxes, scores, classes = detector_utils.detect_objects(
            cv_image, self.inference_graph, self.sess)

        # Draw Bounding box
        detector_utils.draw_box_on_image(
            self.num_objects_detect, self.score_thresh, scores, boxes, classes,
            self.im_width, self.im_height, cv_image)

        # Calculate FPS
        self.num_frames += 1
        elapsed_time = (
            datetime.datetime.now() - self.start_time).total_seconds()
        fps = self.num_frames / elapsed_time

        # Display FPS on frame
        detector_utils.draw_text_on_image(
            "FPS : " + str("{0:.2f}".format(fps)), cv_image)

        # Publish image
        try:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.publisher.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            print(e)

        for i in range(self.num_objects_detect):
            if (scores[i] > self.score_thresh):
                (left, right, top, bottom) = (boxes[i][1] * self.im_width,
                                              boxes[i][3] * self.im_width,
                                              boxes[i][0] * self.im_height,
                                              boxes[i][2] * self.im_height)
                # top left corner of bbox
                p1 = np.array([int(left), int(top)])
                # bottom right corner of bbox
                p2 = np.array([int(right), int(bottom)])
                mid_point = (p1 + p2) / 2
                self.dice_publisher.publish(
                    Point(mid_point[0], mid_point[1], classes[i]))


if __name__ == '__main__':
    classifier()
    rospy.spin()
