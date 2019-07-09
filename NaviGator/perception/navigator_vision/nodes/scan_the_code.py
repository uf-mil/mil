#!/usr/bin/env python
import rospy
import cv2
import numpy as np
from mil_ros_tools import Image_Subscriber, Image_Publisher
from mil_vision_tools import contour_mask, putText_ul, roi_enclosing_points, ImageMux, rect_from_roi
from collections import deque
from navigator_vision import ScanTheCodeClassifier
import tf2_ros
from image_geometry import PinholeCameraModel
import sensor_msgs.point_cloud2
from sensor_msgs.msg import PointCloud2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from navigator_msgs.msg import ScanTheCode


def bbox_countour_from_rectangle(bbox):
    return np.array([[bbox[0][0], bbox[0][1]],
                     [bbox[1][0], bbox[0][1]],
                     [bbox[1][0], bbox[1][1]],
                     [bbox[0][0], bbox[1][1]]])


class ScanTheCodePerception(object):
    def __init__(self):
        self.enabled = False
        self.pattern_pub = rospy.Publisher("/scan_the_code", ScanTheCode, queue_size=3)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.get_params()
        self.last_panel_points_msg = None
        self.sub = Image_Subscriber(self.image_topic, self.img_cb)
        info = self.sub.wait_for_camera_info()
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(info)
        if self.debug:
            self.image_mux = ImageMux(size=(info.height, info.width), shape=(1, 2),
                                      labels=['Result', 'Mask'])
            self.debug_pub = Image_Publisher('~debug_image')
        self.bbox_sub = rospy.Subscriber("stc_led_pts_marshall", PointCloud2, self.panel_points_cb)
        self.classification_list = deque()
        self.enabled = True

    def get_params(self):
        '''
        Set several constants used for image processing and classification
        from ROS params for runtime configurability.
        '''
        self.debug = rospy.get_param('~debug', True)
        self.image_topic = rospy.get_param('~image_topic', '/camera/starboard/image_rect_color')
        self.classifier = ScanTheCodeClassifier()
        self.classifier.train_from_csv()

    def panel_points_cb(self, pc2):
        rospy.loginfo_throttle(10., 'BBox received')
        self.last_panel_points_msg = pc2

    def get_panel_bbox(self):
        if self.last_panel_points_msg is None:
            return None
        try:
            transform = self.tf_buffer.lookup_transform_full(
                self.sub.last_image_header.frame_id,
                self.sub.last_image_header.stamp,
                self.last_panel_points_msg.header.frame_id,
                self.last_panel_points_msg.header.stamp,
                "enu",
                timeout=rospy.Duration(1))
        except tf2_ros.TransformException as e:
            rospy.logwarn(e)
            return None

        transformed_cloud = do_transform_cloud(self.last_panel_points_msg, transform)
        points = np.array(list(sensor_msgs.point_cloud2.read_points(transformed_cloud, skip_nans=True)))
        if len(points) < 4:
            rospy.logwarn('less than 4 points')
            return None
        if self.camera_model is None:
            rospy.logwarn('no camera model')
            return None
        roi = roi_enclosing_points(self.camera_model, points)
        if roi is None:
            rospy.logwarn('No points project into camera.')
            return None
        rect = rect_from_roi(roi)
        ul, br = rect
        xmin, ymin = ul
        xmax, ymax = br

        x_shrink_pixels = 15
        y_shrink_pixels = x_shrink_pixels
        xmin += x_shrink_pixels
        xmax -= x_shrink_pixels
        ymin += y_shrink_pixels
        ymax -= y_shrink_pixels

        new_rect = ((xmin, ymin), (xmax, ymax))

        bbox_contour = bbox_countour_from_rectangle(new_rect)
        return bbox_contour

    def img_cb(self, img):
        if not self.enabled:
            return
        if self.camera_model is None:
            return
        bbox = self.get_panel_bbox()
        if bbox is not None:
            bbox = np.array(bbox, dtype=int)
            debug = contour_mask(bbox, img_shape=img.shape)
            prediction = self.classifier.classify(img, debug)[0]
            label = self.classifier.CLASSES[prediction]
            symbol = label[10]
            if len(self.classification_list) == 0 or self.classification_list[-1] != symbol:
                if len(self.classification_list) >= 5:
                    self.classification_list.popleft()
                self.classification_list.append(symbol)
            text = label + ' | ' + ''.join(self.classification_list)
            scale = 3
            thickness = 2
            putText_ul(debug, text, (0, 0), fontScale=scale, thickness=thickness)
            rospy.loginfo('saw {},  running {}'.format(symbol, text))
            debug = cv2.bitwise_or(img, img, mask=debug)
            self.image_mux[1] = debug
            if len(self.classification_list) == 5 and self.classification_list[0] == 'o' \
               and self.classification_list[4] == 'o' and self.classification_list[1] != 'o' \
               and self.classification_list[2] != 'o' and self.classification_list[3] != 'o':
                pattern = \
                    (self.classification_list[1] + self.classification_list[2] + self.classification_list[3]).upper()
                rospy.loginfo('SAW PATTERN {}!!!!!!!!!!!'.format(pattern))
                self.pattern_pub.publish(ScanTheCode(color_pattern=pattern))
        self.image_mux[0] = img
        if self.debug:
            self.debug_pub.publish(self.image_mux())


if __name__ == '__main__':
    rospy.init_node('scan_the_code_perception')
    s = ScanTheCodePerception()
    rospy.spin()
