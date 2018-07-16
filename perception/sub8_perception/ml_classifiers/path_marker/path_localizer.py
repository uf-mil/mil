#!/usr/bin/python
import os
import cv2
import sys
import rospy
import rospkg
import datetime
import tensorflow as tf
import numpy as np

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from sensor_msgs.msg import RegionOfInterest
from std_msgs.msg import String
from std_srvs.srv import SetBool, SetBoolResponse

rospack = rospkg.RosPack()

# To correctly import utils
sys.path.append(rospack.get_path('sub8_perception') +
                '/ml_classifiers/path_marker/utils')

from utils import detector_utils


class classifier(object):

    def __init__(self):
        '''
        Parameters
        '''
        # Timeout threshold in seconds
        self.time_thresh = rospy.get_param('~time_thresh', 1)
        # Centering Threshold in pixels
        self.centering_thresh = rospy.get_param('~centering_thresh', 50)
        # Color thresholds to find orange, BGR
        self.lower = rospy.get_param('~lower_color_threshold', [0, 30, 100])
        self.upper = rospy.get_param(
            '~upper_color_threshold', [100, 255, 255])
        # Camera topic we are pulling images from for processing
        self.camera_topic = rospy.get_param(
            '~camera_topic', '/camera/down/left/image_rect_color')
        # Number of frames
        self.num_frames = rospy.get_param('~num_frames', 0)
        # Number of objects we detect
        self.num_objects_detect = rospy.get_param('~objects_detected', 2)
        # Mininum confidence score for the detections
        self.score_thresh = rospy.get_param('~score_thresh', 0.1)
        # If we want debug images published or not.
        self.debug = rospy.get_param('~debug', True)
        # Camera image width and height
        self.im_width = rospy.get_param('~im_width', 720)
        self.im_height = rospy.get_param('~im_height', 480)
        # Current time
        self.start_time = datetime.datetime.now()
        # Midpoint of the overall image, in pixels
        self.midpoint = [self.im_width / 2, self.im_height / 2]
        # Whether or not we have centered on an object.
        self.centered = False
        # Whether or not we are enabled
        self.enabled = False

        '''
        Misc Utils, Image Subscriber, and Service Call.
        '''
        # CV bridge for converting from rosmessage to cv_image
        self.bridge = CvBridge()

        # Inference graph and session for tensorflow
        self.inference_graph, self.sess = detector_utils.load_inference_graph()

        # Service Call = the on/off switch for this perception file.
        rospy.Service('~enable', SetBool, self.toggle_search)

        # Subscribes to our image topic, allowing us to process the images
        self.sub1 = rospy.Subscriber(self.camera_topic,
                                     Image, self.img_callback, queue_size=1)

        '''
        Publishers:
        debug_image_pub: publishes the images showing what tensorflow has identified as path markers

        orange_image_pub: publishes our masked image, showing what we see to be orange.

        direction_pub: publishes the proposed direction of the path marker.
        If our mission script reads this and confirms it is a marker,it turns following this direction.
        Either 'left', 'right', or 'none.'

        orange_detection: ensures we are centering on an orange group of pixels.

        path_roi: publishes the region of interest and its coordinates. This is used for debugging purposes.
        '''
        self.debug_image_pub = rospy.Publisher(
            'path_debug', Image, queue_size=1)
        self.orange_image_pub = rospy.Publisher(
            'orange_debug', Image, queue_size=1)
        self.direction_pub = rospy.Publisher(
            'path_direction', String, queue_size=1)
        self.orange_detection = rospy.Publisher(
            'path_orange', String, queue_size=1)
        # self.path_roi_pub = rospy.Publisher(
        # 'path_roi', RegionOfInterest, queue_size=1)

    def toggle_search(self, srv):
        '''
        Callback for standard ~enable service. If true, start
        looking at frames for buoys.
        '''
        if srv.data:
            rospy.loginfo("PATH LOCALIZER: enabled")
            self.enabled = True

        else:
            rospy.loginfo("PATH LOCALIZER: disabled")
            self.enabled = False

        return SetBoolResponse(success=True)

    def check_timestamp(self, msg):
        '''
        Check to see how old the image we are recieving is.
        This is a serious problem considering how long it takes to
        process a single image.
        '''
        if abs(msg.header.stamp.secs - int(rospy.get_time())) > self.time_thresh:
            return True
        else:
            return False

    def img_callback(self, data):
        if not self.enabled:
            # print(self.enabled)
            return None
        if self.check_timestamp(data):
            return None
        try:
            # print('Working')
            cv_image = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        except CvBridgeError as e:
            print(e)

        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

        # Run image through tensorflow graph
        boxes, scores, classes = detector_utils.detect_objects(
            cv_image, self.inference_graph, self.sess)

        # Draw Bounding box
        labelled_image, bbox = detector_utils.draw_box_on_image(
            self.num_objects_detect, self.score_thresh, scores, boxes, classes, self.im_width, self.im_height, cv_image)

        # Find midpoint of the region of interest
        bbox_midpoint = [((bbox[0][1] + bbox[1][1]) / 2),
                         ((bbox[0][0] + bbox[1][0]) / 2)]
        # print(cv_image[int(bbox[0][0]):int(bbox[1][0]),
        # int(bbox[0][1]):int(bbox[1][1])])
        '''     
        Confirm region of interest has orange where the bbox[0] contains the
        topleft coord and bbox[1] contains bottom right
        '''
        # create NumPy arrays from the boundaries
        lower = np.array(self.lower, dtype="uint8")
        upper = np.array(self.upper, dtype="uint8")
        # Run through the mask function, returns all black image if no orange
        check = self.mask_image(cv_image[int(bbox[0][1]):int(bbox[1][1]),
                                         int(bbox[0][0]):int(bbox[1][0])], lower, upper)

        '''
        Find if we are centered on the region of interest, if not display its
        position relative to the center of the camera. Perform the check to see
        if we are looking at an image with orange in it. If not we are done here.
        '''
        check = cv2.cvtColor(check, cv2.COLOR_BGR2GRAY)
        if cv2.countNonZero(check) == 0:
            print('Check Failed.')
            return None
        else:
            # Where [0] is X coord and [1] is Y coord.
            self.find_direction(bbox_midpoint[1], bbox_midpoint[0])

        '''
        Once we center on the region of interest, assuming we are still locked on,
        calculate the curve of the marker. 
        '''
        if self.centered:
            self.find_curve(check)

        # Calculate FPS
        self.num_frames += 1
        elapsed_time = (datetime.datetime.now() -
                        self.start_time).total_seconds()
        fps = self.num_frames / elapsed_time

        # Display FPS on frame
        detector_utils.draw_text_on_image(
            "FPS : " + str("{0:.2f}".format(fps)), cv_image)

        # Publish image
        try:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.debug_image_pub.publish(
                self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            print(e)

    def mask_image(self, cv_image, lower, upper):
        '''
        Mask the image, ensuring we are only looking at orange objects
        '''
        mask = cv2.inRange(cv_image, lower, upper)
        # Remove anything not within the bounds of our mask
        output = cv2.bitwise_and(cv_image, cv_image, mask=mask)

        # gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Blur image so our contours can better find the full shape.
        # blurred = cv2.GaussianBlur(gray, (2, 2), 0)

        if(self.debug):
            try:
                # print(output)
                self.orange_image_pub.publish(
                    self.bridge.cv2_to_imgmsg(output, 'bgr8'))
            except CvBridgeError as e:
                print(e)
        return output

    def find_direction(self, xmid, ymid):
        '''
        Take in region of interest or orange pixel width and height alongside midpoint
        to decide which direction sub should move in to get better lock on the target
        If we are already relatively centered, we tell it to stop moving.
        Recall that the top right corner of the image is (0,0)
        '''
        thresh = self.centering_thresh
        status = None
        width_diff = abs(self.midpoint[0] - xmid)
        height_diff = abs(self.midpoint[1] - ymid)
        if (width_diff > thresh):
            # If x_coord middle is large, it is further right
            if xmid > self.midpoint[0]:
                if(height_diff > thresh):
                    # if y_coord is large it is further down
                    if ymid > self.midpoint[1]:
                        self.centered = False
                        status = 'bot_right'
                    elif ymid < self.midpoint[1]:
                        self.centered = False
                        status = 'top_right'
                else:
                    self.centered = False
                    status = 'right'
            else:
                if(height_diff > thresh):
                    if ymid > self.midpoint[1]:
                        self.centered = False
                        status = 'bot_left'
                    elif ymid < self.midpoint[1]:
                        self.centered = False
                        status = 'top_left'
                else:
                    self.centered = False
                    status = 'left'
        elif(height_diff > thresh):
            if ymid > self.midpoint[1]:
                self.centered = False
                status = 'bot'
            elif ymid < self.midpoint[1]:
                self.centered = False
                status = 'top'
        # If we find that we are within an acceptable distance of the center...
        elif (height_diff < thresh) and (width_diff < thresh):
            self.centered = True
            status = 'center'
        # print('h: ', height_diff)
        # print('w: ', width_diff)
        self.orange_detection.publish(data=status)

    def find_curve(self, blurred):
        '''
        Starts the process of finding the curve of the marker.
        Performs a variety of checks including masking the image and finding contours.
        Pass in the region of interest if availble, otherwise pass in full image.
        '''

        # Compute contours
        cnts = cv2.findContours(blurred.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)

        cnts = cnts[1]
        '''
        We use OpenCV to compute our contours and then begin processing them
        to ensure we are identifying a proper target.
        '''

        shape = ''
        peri_max = 0
        max_x = 0
        max_y = 0
        m_shape = ''
        maxc = [0, 0]
        # print('All Contours: ', cnts)
        for c in cnts:
            # compute the center of the contour, then detect the name of the
            # shape using only the contour
            M = cv2.moments(c)
            if M["m00"] == 0:
                M["m00"] = .000001
            cX = int((M["m10"] / M["m00"]))
            cY = int((M["m01"] / M["m00"]))

            c = c.astype("float")
            c = c.astype("int")

            peri = cv2.arcLength(c, True)
            if peri > peri_max:
                peri_max = peri
                maxc = c
                # print('contour:',  c)
        self.find_extremes(maxc)

    def find_extremes(self, c):
        '''
        This is how we find the direction the marker curves. 
        Passing in the contour, we check all pairs of it, which are pixel coodinates of the contour.
        Once we center on the marker, we look to the midpoint of the ROI.
        We then look at the far left and far right of the image. 
        If the furthest left pixel is higher than the furthest right, we turn right.
        Otherwise we turn left.
        '''
        max_x = 0
        min_x = 100000
        # Quick check to ensure we get a full contour
        if len(c) < 4:
            self.direction_pub.publish(data=None)
            return
        for i in range(len(c)):
            pair = c[i][0]
            # Find the furthest left pixel of the contour
            if pair[0] > max_x:
                max_x = pair[0]
                max_y = pair[1]
            # Find the furthest right pixel of the contour
            elif pair[0] < min_x:
                min_x = pair[0]
                min_y = pair[1]
        # Compare the heights of the pixels
        if max_y > min_y:
            direction = 'right'
        else:
            direction = 'left'
        self.direction_pub.publish(data=direction)

if __name__ == '__main__':
    rospy.init_node('path_localizer', anonymous=False)
    classifier()
    rospy.spin()
