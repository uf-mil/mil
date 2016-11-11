#!/usr/bin/env python
import cv2
import numpy as np

import rospy
import tf
import sensor_msgs.point_cloud2 as pc2
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from sensor_msgs.msg import CameraInfo, Image

from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

import navigator_tools
from navigator_tools import fprint as _fprint


camera_root = "/stereo/right"  # /camera_root/root
rospy.init_node("database_colorer")

ros_t = lambda t: rospy.Duration(t)
fprint = lambda *args, **kwargs: _fprint(title="COLORAMA", time="", *args, **kwargs)

class ImageHolder(object):
    @classmethod
    def from_msg(cls, msg):
        time = msg.header.stamp
        try:
            image = CvBridge().imgmsg_to_cv2(msg)
            return cls(image, time)
        except CvBridgeError, e:
            # Intentionally absorb CvBridge Errors
            rospy.logerr(e)

        return cls()

    def __init__(self, image=None, time=None):
        self.image = image
        self.time = time

    def __repr__(self):
        if self.contains_valid_image:
            return "Image from t={}".format(self.time.to_sec())
        return "Invalid Image"

    @property
    def contains_valid_image(self):
        return self.image is not None and self.time is not None


class ImageHistory(object):
    def __init__(self, image_topic, history=30):
        assert history > 1

        self._history_length = history
        self._images = []
        rospy.Subscriber(image_topic, Image, self._add_image)

    @property
    def newest_image(self):
        if len(self._images) == 0:
            return ImageHolder()
        return max(self._images, key=lambda image: image.time)

    def _add_image(self, msg):
        image = ImageHolder.from_msg(msg)
        self._push(image)

    def _push(self, data):
        '''
            Push to history list and only keep the specified length of history.
            This enforces that all images are valid.
        '''
        if data.contains_valid_image:
            self._images.append(data)
            self._images = self._images[-self._history_length:]

    def get_around_time(self, time, margin=.05):
        '''
            Returns the image that is closest to the specified time.
            Returns `None` if closest image is more than `margin` seconds away.
        '''
        if len(self._images) == 0:
            return ImageHolder()
        closest = min(self._images, key=lambda image: abs(image.time - time))
        if abs(closest.time - time) > ros_t(margin):
            return ImageHolder()  # Return invalid image

        return closest


class DebugImage(object):
    def __init__(self, topic, encoding="bgr8", queue_size=1, prd=1):
        self.bridge = CvBridge()
        self.encoding = encoding
        self.im_pub = rospy.Publisher(topic, Image, queue_size=queue_size)
        self.image = None
        rospy.Timer(ros_t(prd), self.publish)


    def publish(self, *args):
        if self.image is not None:
            try:
                image_message = self.bridge.cv2_to_imgmsg(self.image, self.encoding)
                self.im_pub.publish(image_message)
            except CvBridgeError, e:
                # Intentionally absorb CvBridge Errors
                rospy.logerr(e)


class Colorama(object):
    def __init__(self):
        info_topic = camera_root + "/camera_info"
        image_topic = camera_root + "/image_rect_color"  # Change this to rect on boat

        self.tf_listener = tf.TransformListener()

        db_request = rospy.ServiceProxy("/database/requests", ObjectDBQuery)
        self.make_request = lambda **kwargs: db_request(ObjectDBQueryRequest(**kwargs))

        self.image_history = ImageHistory(image_topic)

        # Wait for camera info, and exit if not found
        try:
            fprint("Waiting for camera info on: '{}'".format(info_topic))
            camera_info_msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=3)
        except rospy.exceptions.ROSException:
            fprint("Camera info not found! Terminating.", msg_color="red")
            rospy.signal_shutdown("Camera not found!")
            return

        fprint("Camera info found!", msg_color="green")
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)

        self.debug = DebugImage("/colorama/debug", prd=.5)

        # These are color mappings from Hue values [0, 360]
        self.color_map = {'red': np.radians(0), 'yellow': np.radians(60),
                          'green': np.radians(120), 'blue': np.radians(240)}

        # Some tunable parameters
        self.update_time = .5  # s
        self.saturation_reject = 50
        self.value_reject = 50
        self.hue_error_reject = .4  # rads

        rospy.Timer(ros_t(self.update_time), self.do_update)

    def do_update(self, *args):
        resp = self.make_request(name='all')

        if resp.found:
            # temp
            time_of_marker = rospy.Time.now() - ros_t(.2)  # header.stamp not filled out
            image_holder = self.image_history.get_around_time(time_of_marker)
            if not image_holder.contains_valid_image:
                return

            header = navigator_tools.make_header(frame="/enu", stamp=image_holder.time) #resp.objects[0].header
            image = image_holder.image
            self.debug.image = np.copy(image)

            cam_tf = self.camera_model.tfFrame()
            fprint("Getting transform between /enu and {}...".format(cam_tf))
            self.tf_listener.waitForTransform("/enu", cam_tf, time_of_marker, ros_t(1))
            t_mat44 = self.tf_listener.asMatrix(cam_tf, header)  # homogenous 4x4 transformation matrix

            for obj in resp.objects:
                if len(obj.points) > 0 and obj.name == "totem":
                    fprint("{} {}".format(obj.id, "=" * 50))

                    print obj.position
                    # Get objects position in camera frame and make sure it's in view
                    object_cam = t_mat44.dot(np.append(navigator_tools.point_to_numpy(obj.position), 1))
                    object_px = map(int, self.camera_model.project3dToPixel(object_cam[:3]))
                    if not self._object_in_frame(object_cam):
                        continue

                    #print object_px

                    points_np = np.array(map(navigator_tools.point_to_numpy, obj.points))
                    # We dont want points below a certain level
                    points_np = points_np[points_np[:, 2] > -2.5]
                    # Shove ones in there to make homogenous points
                    points_np_homo = np.hstack((points_np, np.ones((points_np.shape[0], 1)))).T
                    points_cam = t_mat44.dot(points_np_homo).T
                    points_px = map(self.camera_model.project3dToPixel, points_cam[:, :3])
                    #print points_px

                    roi = self._get_ROI_from_points(points_px)
                    hue = self._get_color_from_ROI(roi, image)
                    c = (0, 0, 0) if hue is None else (int(hue), 255, 255)
                    
                    hsv = np.array([[c]])[:, :3].astype(np.uint8)
                    bgr = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0, 0] 
                    bgr = tuple(bgr.astype(np.uint8).tolist())

                    [cv2.circle(self.debug.image, tuple(map(int, p)), 2, bgr, -1) for p in points_px]
                    cv2.circle(self.debug.image, tuple(object_px), 10, bgr, -1)
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cv2.putText(self.debug.image, str(obj.id), tuple(object_px), font, 1, bgr, 2)


                    print '\n' * 2

    def _get_ROI_from_points(self, image_points):
        # Probably will return a set of contours
        cnt = np.array(image_points).astype(np.float32)

        rect = cv2.minAreaRect(cnt)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        fprint("Drawing rectangle")
        #cv2.drawContours(self.debug.image, [box], 0, (0, 0, 255), 2)
        return box

    def _get_color_from_ROI(self, roi, img):
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.drawContours(mask, [roi], 0, 255, -1)
        bgr = cv2.mean(img, mask)
        # We have to treat that bgr value as a 3d array to get cvtColor to work
        bgr = np.array([[bgr]])[:, :3].astype(np.uint8)
        h, s, v = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0, 0]
        # Now check that s and v are in a good range
        if s < self.saturation_reject or v < self.value_reject:
            fprint("The colors aren't expressive enough. Rejecting.", msg_color='red')
            return None

        # Compute hue error in SO2
        hue_angle = np.radians(h * 2) 
        c = np.cos(hue_angle)
        s = np.sin(hue_angle)
        error = np.inf
        likely_color = 'bazinga'
        for color, h_val in self.color_map.iteritems():
            cg = np.cos(h_val)
            sg = np.sin(h_val)
            this_error = np.abs(np.arctan2(sg*c - cg*s, cg*c + sg*s))
            if this_error < error:
                error = this_error
                likely_color = color

        if error > self.hue_error_reject:
            fprint("Closest color was {} with an error of {} rads (> {}) Rejecting.".format(likely_color, np.round(error, 3), 
                                                                                            self.hue_error_reject), msg_color='red')
            return None

        fprint("Likely color: {} with an hue error of {} rads.".format(likely_color, np.round(error, 3)))
        return np.degrees(self.color_map[likely_color]) / 2.0

    def _object_in_frame(self, object_point):
        """
        Returns if the object is in frame given the centroid of the object.
        `object_point` should be in the camera_model's frame.
        """
        print object_point
        if object_point[2] < 0:
            return False

        px = np.array(self.camera_model.project3dToPixel(object_point))
        resoultion = self.camera_model.fullResolution()
        if np.any([0,0] > px) or np.any(px > resoultion):
            return False

        return True


#rospy.Subscriber("/velodyne_points", pc2.PointCloud2, cb)
# req = rospy.ServiceProxy("/database/requests", ObjectDBQuery)
# resp = req(ObjectDBQueryRequest(name="all"))

# if resp.found:
#     for db_object in resp.objects:
#         if len(db_object.points) > 0:
#             print db_object.points
#         print
c = Colorama()
rospy.spin()
