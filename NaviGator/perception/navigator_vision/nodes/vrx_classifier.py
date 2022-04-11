#!/usr/bin/env python2
import rospy
import cv2
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher, rosmsg_to_numpy
from mil_vision_tools import contour_mask, putText_ul, roi_enclosing_points, ImageMux, rect_from_roi
from collections import deque
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
from vrx_gazebo.msg import Task
from threading import Lock
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from std_srvs.srv import SetBool
import pandas
from PIL import Image

lock = Lock()


def bbox_countour_from_rectangle(bbox):
    return np.array([[bbox[0][0], bbox[0][1]],
                     [bbox[1][0], bbox[0][1]],
                     [bbox[1][0], bbox[1][1]],
                     [bbox[0][0], bbox[1][1]]])

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
        self.task_info_sub = rospy.Subscriber("/vrx/task/info", Task, self.taskinfoSubscriber)
        self.is_perception_task = False
        self.sub = Image_Subscriber(self.image_topic, self.image_cb)
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
        self.last_image = None
        if self.is_training:
            self.enabled = True
        self.queue = []

    @thread_lock(lock)
    def set_enable_srv(self, req):
        self.enabled = req.data
        return {'success': True}

    def image_cb(self, msg):
        self.last_image = msg
        return

    def taskinfoSubscriber(self, msg):
        if not self.is_perception_task and msg.name == "perception":
            self.is_perception_task = True

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

        pixel_cutoff = 50 if self.is_perception_task else 110
        calculate_center = lambda bbox: [(bbox.xmax + bbox.xmin) / 2.0, (bbox.ymax + bbox.ymin) / 2.0]
        abs = lambda x : x if x > 0 else x * -1

        """
        object_ids = {}
        boxes = {}
        classifications = {}
        for a in msg.bounding_boxes:
            center = calculate_center(a)
            boxes[tuple(center)] = None
            classifications[tuple(center)] = a.Class

        #boxes(tuple of center) = id
        #loop through each of the bounding boxes and find the lidar object that is closest
        for a in boxes:
            closest = met_criteria[0]
            print("Met criteria length is {}".format(len(met_criteria)))
            boxes[a] = self.last_objects.objects[closest].id
            for i in met_criteria[1:]:
                print(self.distance(a, pixel_centers[i]) - self.distance(a, pixel_centers[closest]))
                print(abs(distances[i] - distances[closest]))
                if self.distance(a, pixel_centers[i]) < self.distance(a, pixel_centers[closest]) and distances[i] < distances[closest]:
                    closest = i
                    boxes[a] = self.last_objects.objects[i].id
                    print('Better lidar object is {}'.format(boxes[a]))

        for a in boxes:
            print('Object {} classified as {}'.format(boxes[a], classifications[a]))
            cmd = '{}={}'.format(boxes[a], classifications[a])
            self.database_client(ObjectDBQueryRequest(cmd=cmd))"""

        classified = set()

        #for each bounding box,check which buoy is closest to boat within pixel range of bounding box
        for a in msg.bounding_boxes:
            buoys = []

            for i in met_criteria:
                if self.in_rect(pixel_centers[i], a):
                    buoys.append(i)
                
            if len(buoys) > 0:
                closest_to_box = buoys[0]
                closest_to_boat = buoys[0]

                for i in buoys[1:]:
                    if distances[i] < distances[closest_to_boat]:
                        closest_to_box = i
                        closest_to_boat = i

                classified.add(self.last_objects.objects[closest_to_box].id)
                print('Object {} classified as {}'.format(self.last_objects.objects[closest_to_box].id, a.Class))
                cmd = '{}={}'.format(self.last_objects.objects[closest_to_box].id, a.Class)
                self.database_client(ObjectDBQueryRequest(cmd=cmd))

        if not self.is_perception_task:
            return

        for a in met_criteria:
            if self.last_objects.objects[a].id in classified:
                continue
            height = self.last_objects.objects[a].scale.z
            #if pixel_centers[i][0] > 1280 or pixel_centers[i][0] > 720:
            #    return
            if height > 0.45:
                print('Reclassified as white')
                print('Object {} classified as {}'.format(self.last_objects.objects[a].id, "mb_marker_buoy_white"))
                cmd = '{}={}'.format(self.last_objects.objects[a].id, "mb_marker_buoy_white")
                self.database_client(ObjectDBQueryRequest(cmd=cmd))
            else:
                print('Object {} classified as {}'.format(self.last_objects.objects[a].id, "mb_round_buoy_black"))
                cmd = '{}={}'.format(self.last_objects.objects[a].id, "mb_round_buoy_black")
                self.database_client(ObjectDBQueryRequest(cmd=cmd))

        """
        for i in met_criteria:
            boxes = []
            for a in msg.bounding_boxes:
                #if self.in_rect(pixel_centers[i], a):
                boxes.append(a)
            if len(boxes) > 0:
                print('found bounding boxes')
                closest = boxes[0]
                first_center = [(closest.xmax + closest.xmin) / 2.0, (closest.ymax + closest.ymin) / 2.0]
                for a in boxes[1:]:
                    center = [(a.xmax + a.xmin) / 2.0, (a.ymax + a.ymin) / 2.0]
                    if self.distance(pixel_centers[i], center) < self.distance(pixel_centers[i], first_center):
                        closest = a
                        first_center = center
                distance = self.distance(pixel_centers[i], first_center)
                if distance > pixel_cutoff:
                    print('Object {} did not have close enough bounding box with distance {} , using estimation instead'.format(self.last_objects.objects[i].id, distance))
                else:
                    print(distance)
                    print('Object {} classified as {}'.format(self.last_objects.objects[i].id, closest.Class))
                    cmd = '{}={}'.format(self.last_objects.objects[i].id, closest.Class)
                    self.database_client(ObjectDBQueryRequest(cmd=cmd))
                    object_ids[self.last_objects.objects[i].id] = [first_center, pixel_centers[i], closest.Class, distances[i]]
                    continue
            if not self.is_perception_task:
                continue
            height = self.last_objects.objects[i].scale.z
            if pixel_centers[i][0] > 1280 or pixel_centers[i][0] > 720:
                return
            color = self.last_image[int(pixel_centers[i][0]), int(pixel_centers[i][1]),:]
            if height > 0.45:
                object_ids[self.last_objects.objects[i].id] = [None, pixel_centers[i], "mb_marker_buoy_white"]
                print('Reclassified as white')
                print('Object {} classified as {}'.format(self.last_objects.objects[i].id, "mb_marker_buoy_white"))
                cmd = '{}={}'.format(self.last_objects.objects[i].id, "mb_marker_buoy_white")
                self.database_client(ObjectDBQueryRequest(cmd=cmd))
            else:
                object_ids[self.last_objects.objects[i].id] = [None, pixel_centers[i], "mb_round_buoy_black"]
                print('Object {} classified as {}'.format(self.last_objects.objects[i].id, "mb_round_buoy_black"))
                cmd = '{}={}'.format(self.last_objects.objects[i].id, "mb_round_buoy_black")
                self.database_client(ObjectDBQueryRequest(cmd=cmd))
        #used_boxes is a dictionary with the pair of coordinates for the bounding box center as
        #the key and the id for the classified object as the value
        used_boxes = {}

        #object_ids has object id as key and the value is a triple of
        #(bounding box coordinates, object coordinates, object class)
        for classified in object_ids:
            #center := bounding box coordinates
            center = object_ids[classified][0]
            if center is not None:
                center = tuple(center)
            if center is not None and center in used_boxes:
                dist_to_object = self.distance(center, object_ids[classified][1])
                dist_to_stored = self.distance(center, object_ids[used_boxes[center]][1])
                print(dist_to_object)
                print(dist_to_stored)
                if dist_to_object < dist_to_stored and object_ids[classified][3] < object_ids[used_boxes[center]][3]:
                    id = used_boxes[center]
                    used_boxes[center] = classified
                    cmd = '{}={}'.format(classified, object_ids[id][2])
                    self.database_client(ObjectDBQueryRequest(cmd=cmd))
                    cmd = '{}={}'.format(id, "UNKNOWN")
                    self.database_client(ObjectDBQueryRequest(cmd=cmd))
                    object_ids[classified][2] = object_ids[id][2]
            elif center is not None:
                used_boxes[center] = classified"""


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


if __name__ == '__main__':
    rospy.init_node('vrx_classifier')
    c = VrxClassifier()
    rospy.spin()
