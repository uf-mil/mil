#!/usr/bin/env python
import cv2
import numpy as np
import sys
import rospy
import sub8_ros_tools
import image_geometry
from std_msgs.msg import Header
from sub8_msgs.srv import VisionRequest2DResponse, VisionRequest2D
from geometry_msgs.msg import Pose2D
from marker_occ_grid import MarkerOccGrid


class PipeFinder:
    def __init__(self):
        self.last_image = None
        self.last_image_timestamp = None
        self.last_draw_image = None

        self.image_sub = sub8_ros_tools.Image_Subscriber('/down/left/image_raw', self.image_cb)
        self.image_pub = sub8_ros_tools.Image_Publisher("down/left/target_info")

        self.occ_grid = MarkerOccGrid(self.image_sub, grid_res=.05, grid_width=500, grid_height=500,
                                      grid_starting_pose=Pose2D(x=250, y=250, theta=0))

        self.range = sub8_ros_tools.get_parameter_range('/color/channel_guide')
        self.channel_width = sub8_ros_tools.wait_for_param('/vision/channel_width')

        self.pose_service = rospy.Service("vision/channel_marker/2D", VisionRequest2D, self.request_pipe)

        # Occasional status publisher
        self.timer = rospy.Timer(rospy.Duration(.5), self.publish_target_info)

    def publish_target_info(self, *args):
        if self.last_image is None:
            return

        markers = self.find_pipe(np.copy(self.last_image), self.last_image_timestamp)
        self.occ_grid.update_grid(self.last_image_timestamp)
        self.occ_grid.add_marker(markers, self.last_image_timestamp)

        if self.last_draw_image is not None and (markers[0] is not None):
            self.image_pub.publish(self.last_draw_image)

    def image_cb(self, image):
        '''Hang on to last image and when it was taken.'''
        self.last_image = image
        self.last_image_timestamp = self.image_sub.last_image_time

    def get_ncc_scale(self):
        '''Comptue pixel width of the channel marker'''
        _, height = self.occ_grid.get_tf()

        a = np.array(self.occ_grid.cam.project3dToPixel([0.0, self.channel_width, height]))
        b = self.occ_grid.cam.project3dToPixel([0.0, 0.0, height])

        return int(np.nan_to_num(np.linalg.norm(a - b)) / 2.0)

    def ncc(self, image, mean_thresh, scale=5):
        '''Compute normalized cross correlation w.r.t a shadowed pillbox fcn

        TODO:
            Compute the kernel only once
                (Too cheap to need)
        '''
        kernel = np.ones((scale, scale)) * -1
        midpoint = (scale // 2, scale // 2)
        cv2.circle(kernel, midpoint, midpoint[0], 1, -1)

        mean, std_dev = cv2.meanStdDev(image)

        # Check if the scene is brighter than our a priori target
        if mean > mean_thresh:
            kernel = -kernel

        normalized_cross_correlation = cv2.filter2D((image - mean) / std_dev, -1, kernel)
        renormalized = normalized_cross_correlation
        return renormalized

    def find_pipe(self, img, timestamp):
        rows, cols = img.shape[:2]
        if rows == 0:
            return None, None, None

        blur = cv2.GaussianBlur(img, (5, 5), 1000)

        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        channel = 2

        marker_scale = self.get_ncc_scale()
        norc = self.ncc(hsv[::2, ::2, channel], self.range[channel, 0], scale=marker_scale)

        color_mask = cv2.inRange(hsv, self.range[:, 0], self.range[:, 1])

        ncc_mask = cv2.resize(np.uint8(norc >= 0), None, fx=2, fy=2)

        mask = color_mask & ncc_mask
        # todo: use svm
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        blank_img = np.zeros((rows, cols), np.uint8)
        draw_image = np.copy(img)

        if len(contours) > 0:

            # sort contours by area (greatest --> least)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
            cnt = contours[0]  # contour with greatest area

            # This is not to filter incorrectly sized objects, but to ignore speckle noise
            if cv2.contourArea(cnt) > 200:

                cv2.drawContours(blank_img, [cnt], 0, (255, 255, 255), -1)
                cv2.drawContours(draw_image, [cnt], 0, (255, 255, 255), -1)

                # get all coordinates (y,x) of pipe
                why, whx = np.where(blank_img)
                # align coordinates --> (x,y)
                wh = np.array([whx, why])

                # estimate covariance matrix and get corresponding eigenvectors
                cov = np.cov(wh)
                eig_vals, eig_vects = np.linalg.eig(cov)

                # use index of max eigenvalue to find max eigenvector
                i = np.argmax(eig_vals)
                max_eigv = eig_vects[:, i] * np.sqrt(eig_vals[i])

                # flip indices to find min eigenvector
                min_eigv = eig_vects[:, 1 - i] * np.sqrt(eig_vals[1 - i])

                # define center of pipe
                center = np.average(wh, axis=1)

                # define vertical vector (sub's current direction)
                vert_vect = np.array([0.0, -1.0])

                if max_eigv[1] > 0:
                    max_eigv = -max_eigv
                    min_eigv = -min_eigv

                num = np.cross(max_eigv, vert_vect)
                denom = np.linalg.norm(max_eigv) * np.linalg.norm(vert_vect)
                angle_rad = np.arcsin(num / denom)

                cv2.line(draw_image, tuple(np.int0(center)), tuple(np.int0(center + (2 * max_eigv))), (0, 255, 30), 2)
                cv2.line(draw_image, tuple(np.int0(center)), tuple(np.int0(center + (2 * min_eigv))), (0, 30, 255), 2)

                norc_res = cv2.resize(norc, None, fx=2, fy=2)
                draw_image[:, :, 0] = norc_res
                self.last_draw_image = np.copy(draw_image)

                #print center, angle_rad, cv2.contourArea(cnt)
                return center, angle_rad, cv2.contourArea(cnt)

        return None, None, None

    def request_pipe(self, data):
        if self.last_image is None:
            return False  # Fail if we have no images cached

        timestamp = self.last_image_timestamp
        position, orientation, _ = self.find_pipe(self.last_image, timestamp)
        found = (position is not None)

        if not found:
            resp = VisionRequest2DResponse(
                header=Header(
                    stamp=timestamp,
                    frame_id='/down_camera'),
                found=found,
                camera_info=self.image_sub.camera_info,
            )
        else:
            resp = VisionRequest2DResponse(
                pose=Pose2D(
                    x=position[0],
                    y=position[1],
                    theta=orientation
                ),
                max_x=self.last_image.shape[0],
                max_y=self.last_image.shape[1],
                camera_info=self.image_sub.camera_info,
                header=Header(
                    stamp=timestamp,
                    frame_id='/down_camera'),
                found=found
            )
        return resp


def main(args):
    pf = PipeFinder()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('orange_pipe_vision')
    main(sys.argv)
