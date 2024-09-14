#!/usr/bin/env python3
from __future__ import annotations

import bisect
import datetime
import math
import random
from collections import OrderedDict
from threading import Lock

import cv2
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from image_geometry import PinholeCameraModel
from mil_msgs.msg import PerceptionObjectArray
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_ros_tools import Image_Publisher, Image_Subscriber, rosmsg_to_numpy
from mil_vision_tools import ImageMux, rect_from_roi, roi_enclosing_points
from std_srvs.srv import SetBool, Trigger
from tf.transformations import quaternion_matrix
from vision_msgs.msg import Detection2DArray

lock = Lock()


def bbox_countour_from_rectangle(bbox):
    return np.array(
        [
            [bbox[0][0], bbox[0][1]],
            [bbox[1][0], bbox[0][1]],
            [bbox[1][0], bbox[1][1]],
            [bbox[0][0], bbox[1][1]],
        ],
    )


class Classifier:
    # Handle buoys / black totem specially, discrminating on volume as they have the same color
    # The black objects that we have trained the color classifier on
    BLACK_OBJECT_CLASSES = ["buoy", "black_totem"]
    # All the black objects in VRX
    POSSIBLE_BLACK_OBJECTS = ["polyform_a3", "polyform_a5", "polyform_a7"]
    # The average perceceived PCODAR volume of each above object
    BLACK_OBJECT_VOLUMES = [0.3, 0.6, 1.9]
    BLACK_OBJECT_AREA = [0.0, 0.5, 0.0, 0.0]
    TOTEM_MIN_HEIGHT = 0.9

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
        self.database_client = rospy.ServiceProxy("/database/requests", ObjectDBQuery)
        self.is_perception_task = False
        self.prev_objects = OrderedDict()
        self.prev_images = OrderedDict()
        self.sub = Image_Subscriber(self.image_topic, self.image_cb)
        self.bridge = CvBridge()
        self.camera_info = self.sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(self.camera_info)
        if self.debug:
            self.image_mux = ImageMux(
                size=(self.camera_info.height, self.camera_info.width),
                shape=(1, 2),
                labels=["Result", "Mask"],
            )
            self.debug_pub = Image_Publisher("~debug_image")
            self.bbox_pub = Image_Publisher("~bbox_image")
            self.bbox_image = None
        self.last_update_time = rospy.Time.now()
        self.objects_sub = rospy.Subscriber(
            "/pcodar/objects",
            PerceptionObjectArray,
            self.process_objects,
            queue_size=2,
        )
        self.boxes_sub = rospy.Subscriber(
            "/yolov7/detections_model1",
            Detection2DArray,
            self.process_boxes,
        )
        self.enabled_srv = rospy.Service("~set_enabled", SetBool, self.set_enable_srv)
        if self.is_training:
            self.enabled = True
        self.seconds_to_keep = 5
        self.queue = []

        if self.is_simulation:
            self.CLASSES = [
                "red_cylinder",
                "green_cylinder",
                "black_cylinder",
                "white_cylinder",
                "mb_round_buoy_black",
                "mb_round_buoy_orange",
            ]
        else:
            self.CLASSES = [
                "white_cylinder",
                "black_cylinder",
                "red_cylinder",
                "green_cylinder",
            ]

        self.pcodar_reset = rospy.ServiceProxy("/pcodar/reset", Trigger)
        self.pcodar_reset()
        self.clean_timer = rospy.Timer(rospy.Duration(10), self.clean_old_data)

    def clean_old_data(self, event):
        for k, v in self.prev_images.copy().items():
            if k < (rospy.Time.now() - rospy.Duration(self.seconds_to_keep)):
                del self.prev_images[k]
        for k, v in self.prev_objects.copy().items():
            if k < (rospy.Time.now() - rospy.Duration(self.seconds_to_keep)):
                del self.prev_objects[k]

    # GH-880
    # @thread_lock(lock)
    def set_enable_srv(self, req):
        self.enabled = req.data
        return {"success": True}

    def image_cb(self, msg: np.ndarray):
        stamp = self.sub.last_image_header.stamp
        self.prev_images[stamp] = msg

    def get_prev_data(
        self,
        stamp: rospy.Time,
    ) -> tuple[np.ndarray, rospy.Time, PerceptionObjectArray, rospy.Time]:
        image_keys = list(self.prev_images.keys())
        image_index = bisect.bisect_left(image_keys, stamp)
        if image_index < 0:
            rospy.logerr_throttle(
                5,
                "Classifier looking for images that are too early to be in storage...",
            )
            image_index = 0
        if image_index > len(image_keys) - 1:
            rospy.logerr_throttle(
                5,
                "Classifier looking for images that are too late...",
            )
            image_index = len(image_keys) - 1
        image_key = image_keys[image_index]
        relevant_image = self.prev_images[image_keys[image_index]]
        object_keys = list(self.prev_objects.keys())
        object_index = bisect.bisect_left(object_keys, stamp)
        if object_index < 0:
            rospy.logerr_throttle(
                5,
                "Classifier looking for objects that are too early to be in storage...",
            )
            object_index = 0
        if object_index > len(object_keys) - 1:
            rospy.logerr_throttle(
                5,
                f"Classifier looking for objects that are too late {stamp} > {object_keys[-1]}...",
            )
            object_index = len(object_keys) - 1
        object_key = object_keys[object_index]
        relevant_object = self.prev_objects[object_key]
        return relevant_image, image_key, relevant_object, object_key

    def taskinfoSubscriber(self, msg):
        self.is_perception_task = msg.name == "perception"

    def in_frame(self, pixel):
        # TODO: < or <= ???
        return (
            pixel[0] > 0
            and pixel[0] < self.camera_info.width
            and pixel[1] > 0
            and pixel[1] < self.camera_info.height
        )

    # GH-880
    # @thread_lock(lock)
    def process_objects(self, msg: PerceptionObjectArray):
        time = rospy.Time.now()
        self.prev_objects[time] = msg

    def in_rect(self, point, bbox):
        x_buf = self.camera_info.width * 0.04
        y_buf = self.camera_info.height * 0.04
        return bool(
            point[0] >= (bbox.bbox.center.x - bbox.bbox.size_x / 2) - x_buf
            and point[1] >= (bbox.bbox.center.y - bbox.bbox.size_y / 2 - y_buf)
            and point[0] <= (bbox.bbox.center.x + bbox.bbox.size_x / 2 + x_buf)
            and point[1] <= (bbox.bbox.center.y + bbox.bbox.size_y / 2 + y_buf),
        )

    def distance(self, first, second):
        x_diff = second[0] - first[0]
        y_diff = second[1] - first[1]
        return math.sqrt(x_diff * x_diff + y_diff * y_diff)

    def _color_from_label(self, label: str) -> tuple[int, int, int]:
        r = random.Random(hash(label))
        return (
            r.randrange(180, 255),
            r.randrange(180, 255),
            r.randrange(180, 255),
        )

    def _draw_point_vis(self, center: tuple[int, int], label) -> None:
        if self.bbox_image is None:
            return
        color = self._color_from_label(label)
        cv2.circle(self.bbox_image, center, radius=3, color=color, thickness=-1)
        cv2.putText(
            self.bbox_image,
            label,
            center,
            cv2.FONT_HERSHEY_DUPLEX,
            0.5,
            color,
            1,
        )

    def _draw_corner_text(self, label: str, height_from_bottom: int):
        x = 10
        y = self.camera_info.height - 10 - height_from_bottom
        cv2.putText(
            self.bbox_image,
            label,
            (x, y),
            cv2.FONT_HERSHEY_DUPLEX,
            0.9,
            (0, 0, 0),
            1,
        )

    def _draw_bbox_vis(
        self,
        top_left: tuple[int, int],
        bottom_right: tuple[int, int],
        label: str,
        successful: bool = True,
    ):
        if self.bbox_image is None:
            return
        color = (0, 255, 0) if successful else (0, 0, 255)
        cv2.rectangle(self.bbox_image, top_left, bottom_right, color, 1)
        cv2.putText(
            self.bbox_image,
            label,
            (top_left[0], top_left[1] - 18),
            cv2.FONT_HERSHEY_DUPLEX,
            0.5,
            color,
            2,
        )

    def process_boxes(self, msg):
        if not msg.detections:
            return
        if not self.enabled:
            return
        if self.camera_model is None:
            return
        now = rospy.Time.now()
        if now - self.last_update_time < self.update_period:
            return
        self.last_update_time = now
        # Get Transform from ENU to optical at the time of this image
        image_at_time, image_time, objects_at_time, object_time = self.get_prev_data(
            msg.detections[0].source_img.header.stamp,
        )
        self.bbox_image = image_at_time.copy()
        if self.bbox_image is not None:
            image_stamp = datetime.datetime.fromtimestamp(image_time.to_sec())
            object_stamp = datetime.datetime.fromtimestamp(object_time.to_sec())
            current_stamp = datetime.datetime.fromtimestamp(rospy.Time.now().to_sec())
            delay = (current_stamp - image_stamp).total_seconds()
            self._draw_corner_text(
                f"Current time: {current_stamp} (delay: {delay:.2f}s)",
                50,
            )
            self._draw_corner_text(f"Image time used: {image_stamp}", 10)
            self._draw_corner_text(f"LIDAR time used: {object_stamp}", 30)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.sub.last_image_header.frame_id,
                "enu",
                msg.detections[0].source_img.header.stamp,
                # timeout=rospy.Duration(1),
            )
        except Exception:
            rospy.logerr_throttle(
                1,
                "Attempting to process detections, but not enough tf buffer has accumulated.",
            )
            return
        translation = rosmsg_to_numpy(transform.transform.translation)
        rotation = rosmsg_to_numpy(transform.transform.rotation)
        rotation_mat = quaternion_matrix(rotation)[:3, :3]

        # Transform the center of each object into optical frame
        positions_camera = [
            translation + rotation_mat.dot(rosmsg_to_numpy(obj.pose.position))
            for obj in objects_at_time.objects
        ]
        pixel_centers = [
            self.camera_model.project3dToPixel(point) for point in positions_camera
        ]
        distances = np.linalg.norm(positions_camera, axis=1)
        CUTOFF_METERS = 100

        if self.is_perception_task:
            CUTOFF_METERS = 100

        # Get a list of indices of objects who are sufficiently close and can be seen by camera
        met_criteria = []
        for i in range(len(objects_at_time.objects)):
            distance = distances[i]
            if (
                self.in_frame(pixel_centers[i])
                and distance < CUTOFF_METERS
                and positions_camera[i][2] > 0
            ):
                met_criteria.append(i)
                sel_object = objects_at_time.objects[i]
                name = f"({sel_object.id})"
                pixel_x = int(pixel_centers[i][0])
                pixel_y = int(pixel_centers[i][1])
                self._draw_point_vis((pixel_x, pixel_y), name)
        # print 'Keeping {} of {}'.format(len(met_criteria), len(objects_at_time.objects))

        classified = set()

        # for each bounding box,check which buoy is closest to boat within pixel range of bounding box
        failed_detections = []
        for detection in msg.detections:
            buoys = []

            for i in met_criteria:
                if self.in_rect(pixel_centers[i], detection):
                    buoys.append(i)

            if len(buoys) > 0:
                closest_to_box = buoys[0]
                closest_to_boat = buoys[0]

                for i in buoys[1:]:
                    if distances[i] < distances[closest_to_boat]:
                        closest_to_box = i
                        closest_to_boat = i

                label = self.CLASSES[detection.results[0].id]
                classified.add(objects_at_time.objects[closest_to_box].id)
                bbox_left, bbox_right = (
                    round(detection.bbox.center.x - detection.bbox.size_x / 2),
                    round(detection.bbox.center.x + detection.bbox.size_x / 2),
                )
                bbox_bottom, bbox_top = (
                    round(detection.bbox.center.y - detection.bbox.size_y / 2),
                    round(detection.bbox.center.y + detection.bbox.size_y / 2),
                )
                self._draw_bbox_vis(
                    (bbox_left, bbox_top),
                    (bbox_right, bbox_bottom),
                    label,
                )
                # print(
                #    "Object {} classified as {}".format(
                #        objects_at_time.objects[closest_to_box].id,
                #        self.CLASSES[a.results[0].id],
                #    )
                # )
                cmd = f"{objects_at_time.objects[closest_to_box].id}={self.CLASSES[detection.results[0].id]}"
                self.database_client(ObjectDBQueryRequest(cmd=cmd))
            else:
                failed_detections.append(detection)

        for detection in failed_detections:
            bbox_left, bbox_right = (
                round(detection.bbox.center.x - detection.bbox.size_x / 2),
                round(detection.bbox.center.x + detection.bbox.size_x / 2),
            )
            bbox_bottom, bbox_top = (
                round(detection.bbox.center.y - detection.bbox.size_y / 2),
                round(detection.bbox.center.y + detection.bbox.size_y / 2),
            )
            self._draw_bbox_vis(
                (bbox_left, bbox_top),
                (bbox_right, bbox_bottom),
                "failed",
                successful=False,
            )

        if self.bbox_pub.get_num_connections():
            self.bbox_pub.publish(self.bbox_image)

        if not self.is_perception_task:
            return

        for detection in met_criteria:
            if objects_at_time.objects[detection].id in classified:
                continue
            height = objects_at_time.objects[detection].scale.z
            # if pixel_centers[i][0] > 1280 or pixel_centers[i][0] > 720:
            #    return
            if height > 0.45:
                print(
                    "Object {} classified as {}".format(
                        objects_at_time.objects[detection].id,
                        "mb_marker_buoy_white",
                    ),
                )
                cmd = "{}={}".format(
                    objects_at_time.objects[detection].id,
                    "mb_marker_buoy_white",
                )
                self.database_client(ObjectDBQueryRequest(cmd=cmd))
            else:
                print(
                    "Object {} classified as {}".format(
                        objects_at_time.objects[detection].id,
                        "mb_round_buoy_black",
                    ),
                )
                cmd = "{}={}".format(
                    objects_at_time.objects[detection].id,
                    "mb_round_buoy_black",
                )
                self.database_client(ObjectDBQueryRequest(cmd=cmd))

    def get_params(self):
        """
        Set several constants used for image processing and classification
        from ROS params for runtime configurability.
        """
        self.is_training = rospy.get_param("~train", False)
        self.is_simulation = rospy.get_param("/is_simulation", False)
        self.debug = rospy.get_param("~debug", True)
        self.image_topic = rospy.get_param(
            "~image_topic",
            "/camera/front/left/image_color",
        )
        self.model_loc = rospy.get_param("~model_location", "config/model")
        self.update_period = rospy.Duration(1.0 / rospy.get_param("~update_hz", 5))

    def get_box_roi(self, corners):
        roi = roi_enclosing_points(self.camera_model, corners, border=(-10, 0))
        if roi is None:
            rospy.logwarn("No points project into camera.")
            return None
        rect = rect_from_roi(roi)
        bbox_contour = bbox_countour_from_rectangle(rect)
        return bbox_contour

    def get_bbox(self, p, q_mat, obj_msg):
        points = np.zeros((len(obj_msg.points), 3), dtype=np.float)
        for i in range(len(obj_msg.points)):
            points[i, :] = p + q_mat.dot(rosmsg_to_numpy(obj_msg.points[i]))
        return points

    def get_object_roi(self, p, q_mat, obj_msg):
        box_corners = self.get_bbox(p, q_mat, obj_msg)
        return self.get_box_roi(box_corners)


if __name__ == "__main__":
    rospy.init_node("classifier")
    c = Classifier()
    rospy.spin()
