#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher, rosmsg_to_numpy
from mil_vision_tools import contour_mask, putText_ul, roi_enclosing_points, ImageMux, rect_from_roi
from collections import deque
from navigator_vision import VrxColorClassifier
import tf2_ros
from image_geometry import PinholeCameraModel
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
from mil_msgs.msg import PerceptionObjectArray
from std_msgs.msg import Int32
from darknet_ros_msgs.msg import BoundingBoxes
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf.transformations import quaternion_matrix
from mil_tools import thread_lock
import math
from threading import Lock
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from std_srvs.srv import SetBool
import pandas
import tensorflow as tf
from tensorflow.keras import layers, models
from tensorflow import keras
from PIL import Image
from rospkg import RosPack
import os
from bisect import bisect_left

lock = Lock()


def bbox_countour_from_rectangle(bbox):
    return np.array([[bbox[0][0], bbox[0][1]],
                     [bbox[1][0], bbox[0][1]],
                     [bbox[1][0], bbox[1][1]],
                     [bbox[0][0], bbox[1][1]]])

class RunningMean(object):
    def __init__(self, value):
        self.sum = 0
        self.n = 0
        self.add_value(value) 

    def add_value(self, value):
        self.sum += value
        self.n += 1

    @property
    def mean(self):
        return self.sum / self.n


class VotingObject(object):
    def __init__(self):
        self.votes = [0]*6

    def addVote(self, vote):
        self.votes[vote] += 1

    @thread_lock(lock)
    def getHighestScore(self):
        highest = 0
        for i, j in enumerate(self.votes):
            if j > self.votes[highest]:
                highest = i

        return highest

    def empty(self):
        length = 0
        for j in self.votes:
            if j is not 0:
                length += 1
        return length == 0

class VrxClassifier(object):
    # Handle buoys / black totem specially, discrminating on volume as they have the same color
    # The black objects that we have trained the color classifier on
    BLACK_OBJECT_CLASSES = ['buoy', 'black_totem']
    # All the black objects in VRX
    POSSIBLE_BLACK_OBJECTS = ['polyform_a3', 'polyform_a5', 'polyform_a7']
    # The average perceceived PCODAR volume of each above object
    BLACK_OBJECT_VOLUMES = [0.3, 0.6, 1.9]
    BLACK_OBJECT_AREA = [0., 0.5, 0., 0.]
    TOTEM_MIN_HEIGHT = 0.9

    CLASSES = ["mb_marker_buoy_red", "mb_marker_buoy_green", "mb_marker_buoy_black", "mb_marker_buoy_white", "mb_round_buoy_black", "mb_round_buoy_orange"]
    Votes = {}

    def __init__(self):
        self.enabled = False 
        # Maps ID to running class probabilities
        self.object_map = {}
        # Maps ID to mean volume, used to discriminate buoys / black totem
        self.volume_means = {}
        self.area_means = {}
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.get_params()
        self.last_panel_points_msg = None
        self.database_client = rospy.ServiceProxy('/database/requests', ObjectDBQuery)
        self.sub = Image_Subscriber(self.image_topic, self.img_cb)
        self.camera_info = self.sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        if self.debug:
            self.image_mux = ImageMux(size=(self.camera_info.height, self.camera_info.width), shape=(1, 2),
                                      labels=['Result', 'Mask'])
            self.debug_pub = Image_Publisher('~debug_image')
        self.last_objects = None
        self.last_update_time = rospy.Time.now()
        self.objects_sub = rospy.Subscriber('/pcodar/objects', PerceptionObjectArray, self.process_objects, queue_size=2)
        self.boxes_sub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.process_boxes)
        self.enabled_srv = rospy.Service('~set_enabled', SetBool, self.set_enable_srv)
        if self.is_training:
            self.enabled = True
        self.queue = []

    @thread_lock(lock)
    def set_enable_srv(self, req):
        self.enabled = req.data
        return {'success': True}

    def in_frame(self, pixel):
        # TODO: < or <= ???
        return pixel[0] > 0 and pixel[0] < self.camera_info.width and pixel[1] > 0 and pixel[1] < self.camera_info.height

    @thread_lock(lock)
    def process_objects(self, msg):
        self.last_objects = msg

    def in_rect(self, point, bbox):
        if point[0] >= bbox.xmin and point[1] >= bbox.ymin and point[0] <= bbox.xmax and point[1] <= bbox.ymax:
            return True
        else:
            return False

    def distance(self, first, second):
        x_diff = second[0] - first[0]
        y_diff = second[1] - first[1]
        return math.sqrt(x_diff * x_diff + y_diff * y_diff)

    @thread_lock(lock)
    def process_boxes(self, msg):
        if not self.enabled:
            return
        if self.camera_model is None:
            return
        if self.last_objects is None or len(self.last_objects.objects) == 0:
            return
        now = rospy.Time.now()
        if now - self.last_update_time < self.update_period:
            return
        self.last_update_time = now
        # Get Transform from ENU to optical at the time of this image
        transform = self.tf_buffer.lookup_transform(
            self.sub.last_image_header.frame_id,
            "enu",
            self.sub.last_image_header.stamp,
            timeout=rospy.Duration(1))
        translation = rosmsg_to_numpy(transform.transform.translation)
        rotation = rosmsg_to_numpy(transform.transform.rotation)
        rotation_mat = quaternion_matrix(rotation)[:3, :3]

        # Transform the center of each object into optical frame
        positions_camera = [translation + rotation_mat.dot(rosmsg_to_numpy(obj.pose.position))
                            for obj in self.last_objects.objects]
        pixel_centers = [self.camera_model.project3dToPixel(point) for point in positions_camera]
        distances = np.linalg.norm(positions_camera, axis=1)
        CUTOFF_METERS = 30

        # Get a list of indicies of objects who are sufficiently close and can be seen by camera
        met_criteria = []
        for i in xrange(len(self.last_objects.objects)):
            distance = distances[i]
            if self.in_frame(pixel_centers[i]) and distance < CUTOFF_METERS and positions_camera[i][2] > 0:
                met_criteria.append(i)
        # print 'Keeping {} of {}'.format(len(met_criteria), len(self.last_objects.objects))

        for i in met_criteria:
            boxes = []
            for a in msg.bounding_boxes:
                if self.in_rect(pixel_centers[i], a):
                    boxes.append(a)
            if len(boxes) > 0:
                closest = boxes[0]
                first_center = [(closest.xmax - closest.xmin) / 2.0, (closest.ymax - closest.ymin) / 2.0]
                for a in boxes[1:]:
                    center = [(a.xmax - a.xmin) / 2.0, (a.ymax - a.ymin) / 2.0]
                    if self.distance(pixel_centers[i], center) < self.distance(pixel_centers[i], first_center):
                        closest = a
                        first_center = center
                print('Object {} classified as {}'.format(self.last_objects.objects[i].id, closest.Class))
                cmd = '{}={}'.format(self.last_objects.objects[i].id, closest.Class)
                self.database_client(ObjectDBQueryRequest(cmd=cmd))



    def get_params(self):
        '''
        Set several constants used for image processing and classification
        from ROS params for runtime configurability.
        '''
        self.is_training = rospy.get_param('~train', False)
        self.debug = rospy.get_param('~debug', True)
        self.image_topic = rospy.get_param('~image_topic', '/camera/starboard/image_rect_color')
        self.model_loc = rospy.get_param('~model_location','config/model')
        self.update_period = rospy.Duration(1.0 / rospy.get_param('~update_hz', 5))
        self.classifier = VrxColorClassifier()
        self.classifier.train_from_csv()
        rospack = RosPack()
        self.model_loc = os.path.join(rospack.get_path('navigator_vision'), self.model_loc)
        self.model = keras.models.load_model(self.model_loc)

    def get_box_roi(self, corners):
        roi = roi_enclosing_points(self.camera_model, corners, border=(-10, 0))
        if roi is None:
            rospy.logwarn('No points project into camera.')
            return None
        rect = rect_from_roi(roi)
        bbox_contour = bbox_countour_from_rectangle(rect)
        return bbox_contour

    def get_bbox(self, p, q_mat, obj_msg):
        points = np.zeros((len(obj_msg.points), 3) , dtype=np.float)
        for i in range(len(obj_msg.points)):
            points[i, :] = p + q_mat.dot(rosmsg_to_numpy(obj_msg.points[i]))
        return points

    def get_object_roi(self, p, q_mat, obj_msg):
        box_corners = self.get_bbox(p, q_mat, obj_msg)
        return self.get_box_roi(box_corners)

    def update_object(self, object_msg, prediction):
        object_id = object_msg.id

        # Guess the type of object based 
#        most_likely_index = np.argmax(total_probabilities)
#        most_likely_name = self.classifier.CLASSES[most_likely_index]
#        # Unforuntely this doesn't really work'
#        if most_likely_name in self.BLACK_OBJECT_CLASSES:
#            object_scale = rosmsg_to_numpy(object_msg.scale)
#            object_volume = object_scale.dot(object_scale) 
#            object_area = object_scale[:2].dot(object_scale[:2])
#            height = object_scale[2]
#            if object_id in self.volume_means:
#                self.volume_means[object_id].add_value(object_volume)
#                self.area_means[object_id].add_value(object_area)
#            else:
#                self.volume_means[object_id] = RunningMean(object_volume)
#                self.area_means[object_id] = RunningMean(object_area)
#            running_mean_volume = self.volume_means[object_id].mean
#            running_mean_area = self.area_means[object_id].mean
#           
#            if height > self.TOTEM_MIN_HEIGHT:
#                black_guess = 'black_totem'
#            else:
#                black_guess_index = np.argmin(np.abs(self.BLACK_OBJECT_VOLUMES - running_mean_volume))
#                black_guess = self.POSSIBLE_BLACK_OBJECTS[black_guess_index]
#            most_likely_name = black_guess
#            rospy.loginfo('{} current/running volume={}/{} area={}/{} height={}-> {}'.format(object_id, object_volume, running_mean_volume, object_area, running_mean_area, height, black_guess))
#        obj_title = object_msg.labeled_classification
#        probability = class_probabilities[most_likely_index]
#        rospy.loginfo('Object {} {} classified as {} ({}%)'.format(object_id, object_msg.labeled_classification, most_likely_name, probability * 100.))
#        if obj_title != most_likely_name:
#            cmd = '{}={}'.format(object_id, most_likely_name)
#            rospy.loginfo('Updating object {} to {}'.format(object_id, most_likely_name))
#            if not self.is_training:
#                self.database_client(ObjectDBQueryRequest(cmd=cmd))
#        cmd = '{}={}'.format(object_id, self.CLASSES[prediction])
#        self.database_client(ObjectDBQueryRequest(cmd=cmd))
#        rospy.loginfo('Object {} {} classified as {}'.format(object_id, object_msg.labeled_classification, self.CLASSES[prediction]))
        object_msg.labeled_classification = self.CLASSES[prediction]
        
        return self.CLASSES[prediction]

#    @thread_lock(lock)
    def timer_callback(self, event):
        object_id = self.queue.pop()
        if self.Votes[object_id].empty():
            return
        highest = self.Votes[object_id].getHighestScore()
        print('Object {} classified as {}'.format(object_id, self.CLASSES[highest]))
        cmd = '{}={}'.format(object_id, self.CLASSES[highest])
        self.database_client(ObjectDBQueryRequest(cmd=cmd))
        self.Votes[object_id] = VotingObject()
        self.queue.append(object_id)
        rospy.Timer(rospy.Duration(3), self.timer_callback, True)


    @thread_lock(lock)
    def img_cb(self, img):
        return
        if not self.enabled:
            return
        if self.camera_model is None:
            return
        if self.last_objects is None or len(self.last_objects.objects) == 0:
            return
        now = rospy.Time.now()
        if now - self.last_update_time < self.update_period:
            return
        self.last_update_time = now
        # Get Transform from ENU to optical at the time of this image
        transform = self.tf_buffer.lookup_transform(
            self.sub.last_image_header.frame_id,
            "enu",
            self.sub.last_image_header.stamp,
            timeout=rospy.Duration(1))
        translation = rosmsg_to_numpy(transform.transform.translation)
        rotation = rosmsg_to_numpy(transform.transform.rotation)
        rotation_mat = quaternion_matrix(rotation)[:3, :3]

        # Transform the center of each object into optical frame
        positions_camera = [translation + rotation_mat.dot(rosmsg_to_numpy(obj.pose.position))
                            for obj in self.last_objects.objects]
        pixel_centers = [self.camera_model.project3dToPixel(point) for point in positions_camera]
        distances = np.linalg.norm(positions_camera, axis=1)
        CUTOFF_METERS = 15

        # Get a list of indicies of objects who are sufficiently close and can be seen by camera
        met_criteria = []
        for i in xrange(len(self.last_objects.objects)):
            distance = distances[i]
            if self.in_frame(pixel_centers[i]) and distance < CUTOFF_METERS and positions_camera[i][2] > 0:
                met_criteria.append(i)
        # print 'Keeping {} of {}'.format(len(met_criteria), len(self.last_objects.objects))

        rois = [self.get_object_roi(translation, rotation_mat, self.last_objects.objects[i])
                  for i in met_criteria]
        debug = np.zeros(img.shape, dtype=img.dtype)

        if self.is_training:
            training = []


        for i in xrange(len(rois)):
            # The index in self.last_objects that this object is
            index = met_criteria[i]
            # The actual message object we are looking at
            object_msg = self.last_objects.objects[index]
            object_id = object_msg.id
            # Exit early if we could not get a valid roi
            if rois[i] is None:
                rospy.logwarn('Object {} had no points, skipping'.format(object_id))
                continue
            # Form a contour from the ROI
            contour = np.array(rois[i], dtype=int)
            #extract image from contour
            x,y,w,h = cv2.boundingRect(contour)
            image = img[y:y+h,x:x+w]
            image = image[:,:,::-1]
            resized = cv2.resize(image, (100,100))
            arr = np.array(resized, dtype=np.float32)
            arr = np.expand_dims(arr, axis=0)

            # get the color mean features
            #features = np.array(self.classifier.get_features(img, mask)).reshape(1, 9)
            # Predict class probabilites based on color means
            #class_probabilities = self.classifier.feature_probabilities(features)[0]
            prediction = self.model.predict(arr)[0]
            # Use this and previous probabilities to guess at which object this is
            highest = prediction[0]
            loc = 0
            for i, a in enumerate(prediction):
                if a >= highest:
                    highest = a
                    loc = i
            self.update_object(object_msg, loc)
            if(object_id not in self.Votes.keys()):
                self.Votes[object_id] = VotingObject()
                self.queue.append(object_id)
                rospy.Timer(rospy.Duration(3), self.timer_callback, True)
            else:
                self.Votes[object_id].addVote(loc)
            # If training, save this
            if self.is_training and obj_title != 'UNKNOWN':
                classification_index = self.classifier.CLASSES.index(obj_title)
                training.append(np.append(classification_index, features))

            # Draw debug info
            mask = contour_mask(contour, img_shape=img.shape)
            colorful = cv2.bitwise_or(img, img, mask=mask)
            debug = cv2.bitwise_or(debug, colorful)
            scale = 3
            thickness = 2
            center = np.array(pixel_centers[index], dtype=int) - 50
            text = self.CLASSES[loc][9:]
            putText_ul(debug, text, center, fontScale=scale, thickness=thickness)

        #if self.is_training and len(training) != 0:
        #   training = np.array(training)
        #   try:
        #       previous_data = pandas.DataFrame.from_csv(self.classifier.training_file).values
        #       data = np.vstack((previous_data, training))
        #   except Exception as e:
        #       data = training
        #   self.classifier.save_csv(data[:, 1:], data[:, 0])
        #   rospy.signal_shutdown('fdfd')
        #   raise Exception('did something, kev')

        self.image_mux[0] = img
        self.image_mux[1] = debug 
        self.debug_pub.publish(self.image_mux())
        self.last_objects = None
        return


if __name__ == '__main__':
    rospy.init_node('vrx_classifier')
    c = VrxClassifier()
    rospy.spin()
