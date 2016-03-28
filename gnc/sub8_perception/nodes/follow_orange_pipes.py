#!/usr/bin/env python
import cv2
import numpy as np
from tf import transformations
import sys
import rospy
from sensor_msgs.msg import Image
from sub8_msgs.srv import VisionRequestResponse, VisionRequest
import sub8_ros_tools
from geometry_msgs.msg import Pose, PoseStamped
from cv_bridge import CvBridge, CvBridgeError

# define threshold for orange color detection
ORANGE_MIN = np.array([1, 130, 100], np.uint8)
ORANGE_MAX = np.array([23, 255, 255], np.uint8)


class image_converter:
    def __init__(self):
        self.pose_pub = rospy.Publisher("orange_pipe_vision", Pose, queue_size=1)
        self.pose_service = rospy.Service("vision/channel_marker", VisionRequest, self.request_pipe)
        self.image_sub = sub8_ros_tools.ImageSubscriber("forward_camera/image_color", self.image_cb)
        self.last_image = None

    def image_cb(self, image):
        self.last_image = image

    def find_pipe(self, img):
        rows, cols = img.shape[:2]

        blur = cv2.GaussianBlur(img, (5, 5), 0)

        hsv = cv2.cvtColor(blur, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(hsv, ORANGE_MIN, ORANGE_MAX)

        bmask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours, _ = cv2.findContours(bmask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        blank_img = np.zeros((rows, cols), np.uint8)

        if contours:
            # sort contours by area (greatest --> least)
            contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
            cnt = contours[0]  # contour with greatest area
            if cv2.contourArea(cnt) > 1000:  # this value will change based on our depth/the depth of the pool
                rect = cv2.minAreaRect(cnt)   # find bounding rectangle of min area (including rotation)
                box = cv2.cv.BoxPoints(rect)  # get corner coordinates of that rectangle
                box = np.int0(box)            # convert coordinates to ints

                # draw minAreaRect around pipe
                cv2.drawContours(blank_img, [box], 0, (255, 255, 255), -1)

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
                vert_vect = np.array([0, -1 * np.int0(center[1])])

                # calculate angle between vertical and max eigenvector
                num = np.dot(max_eigv, vert_vect)
                denom = np.linalg.norm(max_eigv) * np.linalg.norm(vert_vect)
                angle_rad = np.arccos(num / denom)

                quaternion = transformations.quaternion_from_euler(0.0, 0.0, angle_rad)

                return [center[0], center[1], None], [quaternion[0], quaternion[1], quaternion[2], quaternion[3]]

            else:
                return None

    def request_pipe(self, data):
        if self.last_image is None:
            return False  # Fail if we have no images cached

        pose = self.find_pipe(self.last_image)

        if (pose is not None):
            self.pose_pub.publish(Pose(*pose))
            resp = VisionRequestResponse(
                pose=PoseStamped(
                    pose=Pose(*pose),
                    header=sub8_ros_tools.make_header(frame='/down_camera')
                ),
                found=True
            )
            return resp

        # Indicate a failure to ROS
        else:
            return False


def main(args):
    ic = image_converter()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('orange_pipe_vision', anonymous=True)
    main(sys.argv)