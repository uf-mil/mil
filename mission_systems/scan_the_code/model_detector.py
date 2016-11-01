#!/usr/bin/env python
import roslib
import sys
from collections import deque
from tf import TransformListener
import tf as tf2
import rospy
import cv2
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import navigator_tools as nt
import os
import numpy as np
from scan_the_code_lib.model_tracker import ModelTracker
from navigator_msgs.srv import ScanTheCodeMission, ScanTheCodeMissionResponse, ObjectDBQuery, ObjectDBQueryRequest
from image_geometry import PinholeCameraModel
from debug import Debug
from test_helper import TestHelper
from bag_crawler import BagCrawler


class ScanTheCode:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_cache = deque()
        self.model_tracker = ModelTracker()
        self.pinhole_cam = PinholeCameraModel()
        self.last_cam_info = None
        self.tf = TransformListener()
        self.mission_complete = False
        self.colors = []
        self.scan_the_code = None
        self.on = False
        self.debug = Debug(wait=False)
        self.found_model = False
        self.image_sub = rospy.Subscriber("/stereo/left/image_rect", Image, self.image_cb, queue_size=10)
        self.cam_info_sub = rospy.Subscriber("/stereo/left/camera_info", CameraInfo, self.info_cb, queue_size=10)
        dir_path = os.path.dirname(os.path.realpath(__file__))
        self.img1 = cv2.imread(dir_path + '/train.png', 0)
        self.orb = cv2.ORB(nlevels=3, edgeThreshold=5)
        self.kp1, self.des1 = self.orb.detectAndCompute(self.img1, None)

    def image_cb(self, image):
        if len(self.image_cache) > 50:
            self.image_cache.popleft()
        self.image_cache.append(image)

    def info_cb(self, info):
        self.last_cam_info = info

    def convert_point(self, point):
        K = self.last_cam_info.K
        K = np.array(K).reshape(3, 3)
        pl = point
        pl = K.dot(pl)
        if pl[2] == 0:
            return (0, 0)
        pl[0] /= pl[2]
        pl[1] /= pl[2]
        pl[1] -= 20
        if pl[0] < 0:
            pl[0] = 0
        if pl[1] < 0:
            pl[1] = 0
        return(int(pl[0]), int(pl[1]))

    def get_2d_points(self, points_3d_enu, header, img):
        points_3d = []
        points_2d = []
        t = self.tf.getLatestCommonTime("/stereo_left_cam", "/enu")
        t = rospy.Time.now()
        self.tf.waitForTransform("/stereo_left_cam", "/enu", t, rospy.Duration(3))
        t, r = self.tf.lookupTransform("/stereo_left_cam", "/enu", t)
        transformation = self.tf.fromTranslationRotation(t, r)
        for point in points_3d_enu:
            p = [point.x, point.y, point.z, 1]
            t_p = transformation.dot(p)
            if t_p[3] < 1E-15:
                raise ZeroDivisionError
            t_p[0] /= t_p[3]
            t_p[1] /= t_p[3]
            t_p[2] /= t_p[3]
            points_3d.append(t_p[0:3])

        xmin = 1000
        zmin = 0
        ymin = 1000
        for i, point in enumerate(points_3d):
            if(point[1] < ymin):
                ymin = point[1]
                zmin = point[2]

        buff = .3

        for i, point in enumerate(points_3d):
            if(point[1] < ymin + buff and point[1] > ymin - buff and point[0] < xmin):
                xmin = point[0]
                zmin = point[2]
                # cv2.circle(img, self.convert_point(point), 3, (255, 255, 255), -1)

        xmin += .2
        ymin -= .2

        points_3d = [[xmin, ymin, zmin], [xmin + .5, ymin + .75, zmin], [xmin, ymin + .75, zmin], [xmin + .5, ymin, zmin]]

        for i in range(0, len(points_3d)):
            point = points_3d[i]
            points_2d.append(self.convert_point(point))
        return points_2d

    def get_scan_the_code(self):
        serv = rospy.ServiceProxy("/database/requests", ObjectDBQuery)
        req = ObjectDBQueryRequest()
        req.name = 'scan_the_code'
        scan_the_code = serv(req)
        return scan_the_code.objects[0]

    def get_rectangle(self, img, bounding_rect):
        # preprocess the image, make sure that you have good naming conventions
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 13, 13)
        xmin, ymin, xmax, ymax = bounding_rect
        print gray.shape, ymin, ymax, xmin, xmax
        roi = gray[ymin:ymax, xmin:xmax]
        edges = cv2.Canny(roi, 50, 150, apertureSize=3)
        if edges is None:
            return False, None
        from rect_finder import RectangleFinder
        r = RectangleFinder()
        return r.get_rectangle(edges, roi, self.debug)

    def get_bounding_rect(self, points_2d, img):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, point in enumerate(points_2d):
            if(point[0] < xmin):
                xmin = point[0]
            if(point[0] > xmax):
                xmax = point[0]
            if(point[1] > ymax):
                ymax = point[1]
            if(point[1] < ymin):
                ymin = point[1]
        return xmin, ymin, xmax, ymax

    def start_mission(self):
        while not self.mission_complete:
            if len(self.image_cache) == 0:
                cv2.waitKey(33)
                continue
            cv_image = self.bridge.imgmsg_to_cv2(self.image_cache[len(self.image_cache) - 1], "bgr8").copy()
            img_clone = cv_image.copy()
            img_clone1 = cv_image.copy()

            scan_the_code = self.get_scan_the_code()

            try:
                points_2d = self.get_2d_points(scan_the_code.points, scan_the_code.header, img_clone1)
            except tf2.Exception as exp:
                print exp
                print self.tf.allFramesAsString()
                continue
            except Exception as exp:
                print exp
                continue

            xmin, ymin, xmax, ymax = self.get_bounding_rect(points_2d, cv_image)
            xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
            print xmin, xmax, ymin, ymax

            # DEBUG
            # for i, val in enumerate(points_2d):
            #     cv2.circle(img_clone, (int(val[0]), int(val[1])), 3, (255, 255, 255), -1)
            cv2.rectangle(img_clone, (xmin, ymin), (xmax, ymax), (255, 0, 0))
            cv2.imshow("sup", img_clone)

            succ, rect = self.get_rectangle(cv_image, (xmin, ymin, xmax, ymax))
            if not succ:
                continue

            pnts = []
            print rect
            for p in rect:
                p1 = p[0] + xmin
                p2 = p[1] + ymin
                pnts.append((int(p1), int(p2)))

            self.mission_complete = self.model_tracker.track_models(cv_image, pnts)
            if(self.mission_complete):
                print "MISSION COMPLETE"

            if rect is not None and not self.found_model:
                self.found_model = True
                self.model_tracker.register_model(np.array(pnts, dtype=np.float32), cv_image)

            clone = cv_image.copy()

            for val in pnts:
                cv2.circle(clone, tuple(val), 3, (255, 255, 255), -1)
                self.debug.add_image(clone, "dingle")
            # TODO Get the closest image in the deque

            cv2.waitKey(33)

    def mission_test(self, image, xmin, ymin, xmax, ymax):
        succ, rect = self.get_rectangle(image, (xmin, ymin, xmax, ymax))
        if not succ:
            return

        pnts = []
        print rect
        for p in rect:
            p1 = p[0] + xmin
            p2 = p[1] + ymin
            pnts.append((int(p1), int(p2)))

        self.mission_complete = self.model_tracker.track_models(image, pnts)
        if(self.mission_complete):
            print "MISSION COMPLETE"

        if rect is not None and not self.found_model:
            self.found_model = True
            self.model_tracker.register_model(np.array(pnts, dtype=np.float32), image)

        clone = image.copy()

        for val in pnts:
            cv2.circle(clone, tuple(val), 3, (255, 255, 255), -1)
            self.debug.add_image(clone, "dingle")

    def mission_status(self, req):
        obs = False
        colors = None
        if(len(self.model_tracker.models) > 0):
            obs = True
        if(self.mission_complete):
            colors = self.model_tracker.colors
        found = self.mission_complete
        return ScanTheCodeMissionResponse(obs, found, colors)

    def activate(self, req):
        if(self.on):
            del self.model_tracker
            self.model_tracker = ModelTracker()
            self.mission_complete = False
            self.colors = []
            self.on = False
        if(not self.on):
            self.on = True
            self.start_mission()
        return ScanTheCodeMissionResponse(None, None, None)


def main(args):
    rospy.init_node('model_detector', anonymous=True)
    ic = ScanTheCode()
    status = rospy.Service('/vision/scan_the_code_status', ScanTheCodeMission, ic.mission_status)
    activate = rospy.Service('/vision/scan_the_code_activate', ScanTheCodeMission, ic.activate)

    ic.activate(None)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()


def main_test(args):
    rospy.init_node('model_detector', anonymous=True)
    s = ScanTheCode()
    t = TestHelper()
    mine = t.get_roi('stc.dat')
    bc = BagCrawler('/home/tess/bags/stc/platform.bag')
    crawl = bc.crawl('/stereo/left/image_raw')
    crawl.next()
    for image in crawl:
        x, y, w, h = mine.next()
        xmin, ymin, xmax, ymax = int(x) - int(w) / 2, int(y) - int(h) / 2 + 5, int(x) + int(w) / 2, int(y) + int(h) / 2
        s.mission_test(image, xmin, ymin, xmax, ymax)


if __name__ == '__main__':
    main(sys.argv)
