#!/usr/bin/python
import cv2
import sys
import rospy
import rospkg
import datetime
import numpy as np
import argparse
import os
import image_geometry
import mil_tools
import tf
import multiprocessing
from geometry_msgs.msg import Point
from mil_vision_tools import VisionNode, create_object_msg
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryResponse
from mil_msgs.msg import PerceptionObject, ObjectInImage, ObjectsInImage
from sensor_msgs.msg import RegionOfInterest
from utils import detector_utils  # noqa
from mil_ros_tools import numpy_to_point2d
import rospkg
from CLAHE_Processing import CLAHEGenerator

rospack = rospkg.RosPack()

class classifier(object):

    def __init__(self):
        '''
        Parameters
        '''
        parser = argparse.ArgumentParser(description='TensorFlow Object Detection',
                                        usage='''Default parameters: None''')
        parser.add_argument('-g', '--garlic', action='store_true',
                            help='Sets up Network for Bin Dropper')
        parser.add_argument('-v', '--vamp', action='store_true', help='Sets up Network for the Buoy')
        parser.add_argument('-s', '--stake', action='store_true',
                            help='Sets up Network for the Torpedo Board')
        args= parser.parse_args()

        self.generator = CLAHEGenerator()

        self.tf_listener = tf.TransformListener()

        self.garlic = args.garlic
        self.vamp = args.vamp
        self.stake = args.stake

        if self.garlic:
            self.target = 'garlic'
            self.classes = 1
            self.see_sub = mil_tools.Image_Subscriber(
              topic="/camera/down/image_rect_color", callback=self.img_callback)
        elif self.vamp:
            self.target = 'vamp'
            self.classes = 4
            self.see_sub = mil_tools.Image_Subscriber(
              topic="/camera/front/left/image_rect_color", callback=self.img_callback)
        else:  
            self.target = 'stake'
            self.classes = 1
            self.see_sub = mil_tools.Image_Subscriber(
              topic="/camera/front/left/image_rect_color", callback=self.img_callback)
        # Number of frames
        self.num_frames = rospy.get_param('~num_frames', 0)
        # Number of objects we detect
        self.num_objects_detect = rospy.get_param('~objects_detected', 1)
        # Mininum confidence score for the detections
        self.score_thresh = rospy.get_param('~score_thresh', 0.99)
        # If we want debug images published or not.
        self.debug = rospy.get_param('~debug', True)
        # Camera image width and height
        self.im_width = 1920
        self.im_height = 1080
        # Current time
        self.start_time = datetime.datetime.now()
        '''
        Misc Utils, Image Subscriber, and Service Call.
        '''
        # CV bridge for converting from rosmessage to cv_image
        self.bridge = CvBridge()

        self.inference_graph, self.sess = detector_utils.load_inference_graph(self.target, self.classes)

        # Subscribes to our image topic, allowing us to process the images
        # self.sub1 = rospy.Subscriber(
        #    self.camera_topic, Image, self.img_callback, queue_size=1)
        self.see_frame_counter = 0
        self.see_camera_info = self.see_sub.wait_for_camera_info()
        self.see_img_geom = image_geometry.PinholeCameraModel()
        self.see_img_geom.fromCameraInfo(self.see_camera_info)  
        '''
        Publishers:
        debug_image_pub: publishes the images showing what tensorflow has
        identified as path markers
        bbox_pub: publishes the bounding boxes. 
        '''
        self.debug_image_pub = rospy.Publisher(
            'path_debug', Image, queue_size=1)
        self.bbox_pub = rospy.Publisher('bbox_pub', Point, queue_size=1)
        self.roi_pub = rospy.Publisher('roi_pub', RegionOfInterest, queue_size=1)
        self.dictionary = {}

    def check_timestamp(self):
        '''
        Check to see how old the image we are recieving is.
        This is a serious problem considering how long it takes to
        process a single image.
        '''
        see = self.see_sub.last_image_header
        if abs(see.stamp.secs - int(rospy.get_time())
               ) > self.time_thresh:
            return True
        else:
            return False

    def img_callback(self, data):
        self.parse_label_map()
        # if self.check_timestamp():
           # return None
        
        cv_image = data
        cv_image = self.generator.CLAHE(cv_image)
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        self.im_height, self.im_width, channels  = cv_image.shape
        print("Height ", self.im_height, " Width: ", self.im_width)
        #  cv_image = cv2.resize(cv_image, (256, 144))
        # Run image through tensorflow graph
        boxes, scores, classes = detector_utils.detect_objects(
            cv_image, self.inference_graph, self.sess)

        # Draw Bounding box
        labelled_image, bbox = detector_utils.draw_box_on_image(
            self.num_objects_detect, self.score_thresh, scores, boxes, classes,
            self.im_width, self.im_height, cv_image, self.target)

        # Calculate FPS
        self.num_frames += 1
        elapsed_time = (
            datetime.datetime.now() - self.start_time).total_seconds()
        fps = self.num_frames / elapsed_time

        # Display FPS on frame
        detector_utils.draw_text_on_image(
            "FPS : " + str("{0:.2f}".format(fps)), cv_image)
        print("bbox:", bbox)  
        if len(bbox) > 0:
          pointx = (bbox[0][0] + bbox[1][0]) /2 
          pointy = (bbox[0][1] + bbox[1][1]) /2
          pointxdist = abs(bbox[0][0] - bbox[1][0])
          pointydist = abs(bbox[0][1] - bbox[1][1])
          print(pointxdist)
          msg = Point(x=pointx, y=pointy, z=self.see_sub.last_image_time.to_sec())
          print("X: ", pointx, "Y: ", pointy, "TIMESTAMP: ", msg.z)
          self.bbox_pub.publish(msg)
          roi = RegionOfInterest(x_offset=int(bbox[0][0]), y_offset=int(bbox[0][1]), height = int(pointydist), width=int(pointxdist))
          self.roi_pub.publish(roi)
        
          
        # Publish image
        try:
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
            self.debug_image_pub.publish(
                self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except CvBridgeError as e:
            print(e)

    def parse_label_map(self):
        labelmap = rospack.get_path('sub8_perception')+'/datasets/'+ self.target + '.pbtxt'
        with open(labelmap) as f:
            txt = f.read()
            labels = []
            ids = []
            # print(txt)
            # txt = txt[2, :]
            full_split = [s.strip().split(': ') for s in txt.splitlines()]
            # print(full_split)
            full_split = full_split[1:]
            # try:
            for i in full_split:
                if len(i) < 2:
                    continue
                if isinstance(i[1], str):
                    if i[1].isdigit():
                        # print(i[1])
                        ids.append(int(i[1]))
                    else:
                        print(i[1].strip("'"))
                        labels.append(i[1].strip("'"))
                else:
                    print(
                        "Error, incorrect key located in labelmap. Should be only id or name. Instead found: ", i[1])
            # except:
                # print("It errored! Whomp whomp. ", i)
        self.dictionary = dict(zip(ids, labels))

if __name__ == '__main__':
    rospy.init_node('localizer', anonymous=False)
    classifier()
    rospy.spin()
