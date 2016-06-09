#!/usr/bin/env python
import cv2
import numpy as np
import sys
import rospy
import image_geometry
import sub8_ros_tools
from sub8_msgs.srv import VisionRequest2DResponse, VisionRequest2D
from std_msgs.msg import Header
from geometry_msgs.msg import Pose2D


def contour_sort(l):
    """Sort contours by area largest to smallest."""
    length = len(l)
    if length <= 1:
        return l
    else:
        pivot = l.pop(int(length / 2))
        less, more = [], []
        for x in l:
            if cv2.contourArea(x) >= cv2.contourArea(pivot):
                less.append(x)
            else:
                more.append(x)
        return contour_sort(less) + [pivot] + contour_sort(more)


def evaluate_bin(roi):
    """Check for orangeness."""
    b1 = 53
    g1 = 60
    r1 = 60
    b2 = 120
    g2 = 145
    r2 = 123
    lower_value = np.array([b1, g1, r1], np.uint8)
    upper_value = np.array([b2, g2, r2], np.uint8)
    temp = np.array(0)
    mask = cv2.inRange(roi, lower_value, upper_value)
    bimg = cv2.bitwise_or(mask, temp)
    orangeness = bimg.mean()
    return orangeness


class BinFinder:

    def __init__(self):
        rospy.sleep(1.0)
        self.bin_type = None
        self.last_image = None
        self.last_draw_image = None
        self.last_image_time = None
        self.camera_model = None
        self.pose_service = rospy.Service('vision/bin/2D', VisionRequest2D, self.request_bin)
        self.image_sub = sub8_ros_tools.Image_Subscriber('/down_camera/image_rect_color', self.image_cb)
        self.image_pub = sub8_ros_tools.Image_Publisher('/vision/bin_2d/target_info')

        # Occasional status publisher
        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_target_info)

        self.bins = {
            'orange': '/color/bin/orange',
            'norange': '/color/bin/norange',
        }

    def request_bin(self, srv):
        self.bin_type = srv.target_name
        if (self.last_image != None):
            print 'requesting', srv
            response = self.find_single_bin(np.copy(self.last_image), srv.target_name)

            if response is False:
                print 'did not find'
                resp = VisionRequest2DResponse(
                    header=sub8_ros_tools.make_header(frame='/down'),
                    found=False
                )

            else:
                # Fill in
                center, radius = response
                resp = VisionRequest2DResponse(
                    header=Header(stamp=self.last_image_time, frame_id='/down'),
                    pose=Pose2D(
                        x=center[0],
                        y=center[1],
                        theta=radius
                    ),
                    max_x=self.last_image.shape[0],
                    max_y=self.last_image.shape[1],
                    camera_info=self.image_sub.camera_info,
                    found=True
                )
            return resp

    def publish_target_info(self, *args):
        if self.last_image is None:
            return
        self.find_bins(np.copy(self.last_image), self.bin_type)
        if self.last_draw_image is not None:
            self.image_pub.publish(self.last_draw_image)

    def image_cb(self, image):
        '''Hang on to last image'''
        self.last_image = image
        self.last_image_time = self.image_sub.last_image_time
        if self.camera_model is None:
            if self.image_sub.camera_info is None:
                return

            self.camera_model = image_geometry.PinholeCameraModel()
            self.camera_model.fromCameraInfo(self.image_sub.camera_info)

    def find_single_bin(self, img, bin_type):
        """Find the bins and their orientations."""
        assert bin_type in self.bins[bin_type], "Bins_2d does not know bin color: {}".format(bin_type)
        if img is not None:

            debug_image = np.copy(img)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            zeros = np.array([0])
            mask = cv2.inRange(img, 127, 255)
            img = cv2.bitwise_or(mask, zeros)
            contours, hierarchy = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            contours = contour_sort(contours)

            """This finds the bins and looks for the one that is orange or is not
               orange. Each bin is given an orangeness rating and either the most 
               or least orange bin is selected.
            """
            if len(contours) > 0:
                bins = 4
                if (self.bin_type == 'orange'):
                    orangeness = 0
                else:
                    orangeness = 100000
                if len(contours) < 4:
                    bins = len(contours)
                for i in range(0, bins):
                    x, y, w, h = cv2.boundingRect(contours[i])
                    roi = debug_image[y: y + h, x: x + w]
                    temp = evaluate_bin(roi)
                    if ((orangeness > temp and self.bin_type == 'norange') 
                    or (orangeness < temp and self.bin_type == 'orange')):
                        orangeness = temp
                        M = cv2.moments(contours[i])
                        cx = int(M['m10'] / M['m00'])
                        cy = int(M['m01'] / M['m00'])
                        img_h, img_w, _ = np.shape(debug_image)
                        point = (cx, cy)
                        (_, _), (_, _), rad = cv2.fitEllipse(contours[i])
                    cv2.rectangle(debug_image, (x, y), (x + w, y + h), (127), 2)
                    ellipse = cv2.fitEllipse(contours[i])
                    cv2.ellipse(debug_image, ellipse, (170), 2)
                cv2.circle(debug_image, point, 5, (0, 0, 255), -1)
                point = [cx - (img_w / 2), cy - (img_h / 2)]
                tuple_center = (point[0], point[1], 0)
            self.last_draw_image = debug_image
            rad = ((rad - 180) * np.pi) / 180.0
            return tuple_center, rad

    def find_bins(self, img, srv):
        draw_image = np.copy(img)
        result = self.find_single_bin(img, self.bin_type)

def main(args):
    bf = BinFinder()
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('bin_vision')
    main(sys.argv)
