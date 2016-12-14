#!/usr/bin/env python
from __future__ import division

import cv2
import numpy as np
from collections import deque
from copy import deepcopy

import rospy
import tf
import tf.transformations as trns

import sensor_msgs.point_cloud2 as pc2
from navigator_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest, ColorRequest, ColorRequestResponse
from navigator_msgs.msg import ColoramaDebug
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry

from cv_bridge import CvBridge, CvBridgeError
from image_geometry import PinholeCameraModel

import navigator_tools
from navigator_tools import fprint as _fprint


camera_root = "/stereo/right"  # /camera_root/root

ros_t = lambda t: rospy.Duration(t)
fprint = lambda *args, **kwargs: _fprint(title="COLORAMA", time="", *args, **kwargs)
p2np = navigator_tools.point_to_numpy

class ImageHolder(object):
    @classmethod
    def from_msg(cls, msg):
        time = msg.header.stamp
        fprint("Got image! {}".format(time.to_sec()))
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

    def get_around_time(self, time, margin=.5, timeout=.5):
        '''
            Returns the image that is closest to the specified time.
            Returns `None` if closest image is more than `margin` seconds away.
        '''
        start_time = rospy.Time.now()
        while rospy.Time.now() - start_time <= ros_t(timeout):
            if len(self._images) == 0:
                continue

            closest = min(self._images, key=lambda image: abs(image.time - time))
            if abs(closest.time - time) > ros_t(margin):
                continue  # Return invalid image

            return closest
        
        return ImageHolder()

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


class Observation(object):
    history_length = 100  # Default to 100
    
    @classmethod
    def as_message(self):
        msg = ColoramaDebug()
        msg.num_observations = len(self.hues)

        msg.mean_value = np.mean(self.values) 
        msg.hues = self.hues

    def __init__(self):
        self.hues = deque([], maxlen=self.history_length)
        self.values = deque([], maxlen=self.history_length) 
        self.q_errs = deque([], maxlen=self.history_length)
        self.dists = deque([], maxlen=self.history_length)
    
    def __len__(self):
        return len(self.hues)
   
    def __iadd__(self, (hue, value, dist, q_diff)):
        """
        Add observations like:
            >>> co += [0.65, 0.4, ..., 0.1]
        """
        self.hues.append(hue)
        self.values.append(value)
        self.dists.append(dist)
        self.q_errs.append(q_diff)

        return self
  
    def __repr__(self):
        _str = 'hues: {}\nvalues: {}\ndists: {}\nq_errs: {}'
        return _str.format(*np.round(map(np.array, [self.hues, self.values, self.dists, self.q_errs]), 3))

    def extend(self, (hues, values, dists, q_diffs)):
        """Add lists of data in at once"""
        self.hues.extend(hues)
        self.values.extend(values)
        self.dists.extend(dists)
        self.q_errs.extend(q_diffs)

    def compute_confidence(self, (value_w, dist_w, q_diff_w), get_conf=False, **kwargs):
        """Don't try to compute weights with bad data (too few samples)"""

        # Get some value information
        v_u = kwargs.get('v_u', 230)
        v_sig = kwargs.get('v_sig', 30)
        dist_sig = kwargs.get('dist_sig', 70)
        q_sig = kwargs.get('q_sig', 1.3)

        # Compute data required before applying weights
        guass = self._guass
        value_errs = guass(self.values, v_u, v_sig)
        dists = guass(self.dists, 5, dist_sig)
        q_diffs = guass(self.q_errs, 0, q_sig)

        # Normalize weights
        w_norm = value_w + dist_w + q_diff_w
        value_w /= w_norm
        dist_w /= w_norm
        q_diff_w /= w_norm

        #print "1", hue_std_w, np.round(hue_std, 2), hue_std_w * hue_std
        #print "2", value_w, np.round(value_errs, 2), value_w * value_errs
        #print "3", dist_w, np.round(dists, 2), dist_w * dists
        #print "4", q_diff_w, np.round(q_diffs, 2), q_diff_w * q_diffs

        # Compute normalized confidence
        c = value_w * value_errs + dist_w * dists + q_diff_w * q_diffs
        if get_conf:
            return c, [value_errs, dists, q_diffs]

        return c
    
    def _guass(self, data, mean, sig):
        return np.exp(-np.power(np.array(data).astype(np.float64) - mean, 2) / (2 * np.power(sig, 2)))


class Colorama(object):
    def __init__(self):
        info_topic = camera_root + "/camera_info"
        image_topic = camera_root + "/image_rect_color"

        self.tf_listener = tf.TransformListener()
        self.status_pub = rospy.Publisher("/database_color_status", ColoramaDebug, queue_size=1)

        self.odom = None 
        set_odom = lambda msg: setattr(self, "odom", navigator_tools.pose_to_numpy(msg.pose.pose))
        rospy.Subscriber("/odom", Odometry, set_odom)
        fprint("Waiting for odom...")
        while self.odom is None and not rospy.is_shutdown():
            rospy.sleep(1)
        fprint("Odom found!", msg_color='green')

        db_request = rospy.ServiceProxy("/database/requests", ObjectDBQuery)
        self.make_request = lambda **kwargs: db_request(ObjectDBQueryRequest(**kwargs))

        self.image_history = ImageHistory(image_topic)

        # Wait for camera info, and exit if not found
        fprint("Waiting for camera info on: '{}'".format(info_topic))
        while not rospy.is_shutdown():
            try:
                camera_info_msg = rospy.wait_for_message(info_topic, CameraInfo, timeout=3)
            except rospy.exceptions.ROSException:
                rospy.sleep(1)
                continue
            break

        fprint("Camera info found!", msg_color="green")
        self.camera_model = PinholeCameraModel()
        self.camera_model.fromCameraInfo(camera_info_msg)

        self.debug = DebugImage("/colorama/debug", prd=.5)

        # These are color mappings from Hue values [0, 360]
        self.color_map = {'red': np.radians(0), 'yellow': np.radians(60),
                          'green': np.radians(120), 'blue': np.radians(200)}

        # RGB map for database setting
        self.database_color_map = {'red': (255, 0, 0), 'yellow': (255, 255, 0), 'green': (0, 255, 0), 'blue': (0, 0, 255)}

        # ========= Some tunable parameters ===================================
        self.update_time = 1  # s

        # Observation parameters
        self.saturation_reject = 20 # Reject color obs with below this saturation
        self.value_reject = 50      # Reject color obs with below this value
        self.height_remove = 0.4    # Remove points that are this percent down on the object (%)
                                    # 1 keeps all, .4 removes the bottom 40%
        # Update parameters
        self.history_length = 100   # How many of each color to keep
        self.min_obs = 5            # Need atleast this many observations before making a determination
        self.conf_reject = .5       # When to reject an observation based on it's confidence

        # Confidence weights
        self.v_factor = 0.6         # Favor value values close to the nominal value
        self.v_u = 200              # Mean of norm for variance error
        self.v_sig = 60             # Sigma of norm for variance error
        self.dist_factor = 0.3      # Favor being closer to the totem
        self.dist_sig = 30          # Sigma of distance (m)
        self.q_factor = 0           # Favor not looking into the sun
        self.q_sig = 1.2            # Sigma of norm for quaternion error (rads)
        
        # Maps id => observations
        self.colored = {}

        rospy.Timer(ros_t(self.update_time), self.do_observe)

    def _compute_average_angle(self, angles, weights):
        """
        Returns weighted average of angles.
        Tends to break down with too many angles 180 degrees of each other - try not to do that.
        """
        angles = np.array(angles)
        if np.linalg.norm(weights) == 0:
            return None

        w = weights / np.linalg.norm(weights)
        s = np.sum(w * np.sin(angles))
        c = np.sum(w * np.cos(angles))
        avg_angle = np.arctan2(s, c)
        return avg_angle
   
    def _get_quaternion_error(self, q, target_q):
        """
        Returns an angluar differnce between q and target_q in radians
        """
        dq = trns.quaternion_multiply(np.array(target_q), trns.quaternion_inverse(np.array(q)))
        return 2 * np.arccos(dq[3])

    def _get_closest_color(self, hue_angle):
        """
        Returns a pair of the most likely color and the radian error associated with that prediction
            `hue_angle` := The radian value associated with hue [0, 2*pi] 
        Colors are found from `self.color_map`
        """
        c = np.cos(hue_angle)
        s = np.sin(hue_angle)
        error = np.inf
        likely_color = 'undef'
        for color, h_val in self.color_map.iteritems():
            cg = np.cos(h_val)
            sg = np.sin(h_val)
            # We need a signed error for some math later on so no absolute value
            this_error = np.arctan2(sg*c - cg*s, cg*c + sg*s)
            if np.abs(this_error) < np.abs(error):
                error = this_error
                likely_color = color

        fprint("Likely color: {} with an hue error of {} rads.".format(likely_color, np.round(error, 3)))
        return [likely_color, error] 

    def do_estimate(self, totem_id):
        """Calculates best color estimate for a given totem id"""
        fprint("DOING ESTIMATE", msg_color='blue') 
        if totem_id not in self.colored:
            fprint("Totem ID {} not found!".format(totem_id), msg_color='red')
            return None
       
        t_color = self.colored[totem_id]
        
        if len(t_color) < self.min_obs:
            fprint("Only {} observations. {} required.".format(len(t_color), self.min_obs), msg_color='red')
            return None

        kwargs = {'v_u': self.v_u, 'v_sig': self.v_sig, 'dist_sig': self.dist_sig, 
                  'q_factor': self.q_factor, 'q_sig': self.q_sig}

        w, weights = t_color.compute_confidence([self.v_factor, self.dist_factor, self.q_factor], True, **kwargs)
        fprint("CONF: {}".format(w))
        if np.mean(w) < self.conf_reject:
            return None
        
        hue_angles = np.radians(np.array(t_color.hues) * 2) 
        angle = self._compute_average_angle(hue_angles, w)
        color = self._get_closest_color(angle)
        
        msg = t_color.as_message 
        msg.id = totem_id
        msg.confidence = w
        msg.labels = ["value_errs", "dists", "q_diffs"]
        msg.weights = weights
        msg.color = colors[0]
        msg.est_hues = angle * 2
        msg.hues = np.array(t_color.hues) * 2
        self.status_pub.publish(msg)

        fprint("Color: {}".format(color[0]))
        return color
    
    def got_request(self, req):
        # Threading blah blah something unsafe
        colored_ids = [_id for _id, color_err in self.colored.iteritems() if self.valid_color(_id) == req.color]
        
        fprint("Colored IDs: {}".format(colored_ids), msg_color='blue')
        print '\n' * 50
        if len(colored_ids) == 0:
            return ColorRequestResponse(found=False)
        
        return ColorRequestResponse(found=True, ids=colored_ids)

    def do_observe(self, *args):
        resp = self.make_request(name='totem')
        
        # If atleast one totem was found start observing
        if resp.found:
            # Time of the databse request
            time_of_marker = resp.objects[0].header.stamp# - ros_t(1)
            fprint("Looking for image at {}".format(time_of_marker.to_sec()), msg_color='yellow')
            image_holder = self.image_history.get_around_time(time_of_marker)
            if not image_holder.contains_valid_image:
                t = self.image_history.newest_image.time
                if t is None:
                    fprint("No images found.")
                    return
                
                fprint("No valid image found for t={} ({}) dt: {}".format(time_of_marker.to_sec(), t.to_sec(), (rospy.Time.now() - t).to_sec()), msg_color='red')
                return
            header = navigator_tools.make_header(frame='/enu', stamp=image_holder.time)
            image = image_holder.image
            self.debug.image = np.copy(image)

            cam_tf = self.camera_model.tfFrame()
            try:
                fprint("Getting transform between /enu and {}...".format(cam_tf))
                self.tf_listener.waitForTransform("/enu", cam_tf, time_of_marker, ros_t(1))
                t_mat44 = self.tf_listener.asMatrix(cam_tf, header)
            except tf.ExtrapolationException as e:
                fprint("TF error found and excepted: {}".format(e))
                return

            for obj in resp.objects:
                if len(obj.points) <= 1:
                    fprint("No points in object")
                    continue

                fprint("{} {}".format(obj.id, "=" * 50))
                
                # Get object position in px coordinates to determine if it's in frame
                object_cam = t_mat44.dot(np.append(p2np(obj.position), 1))
                object_px = map(int, self.camera_model.project3dToPixel(object_cam[:3]))
                if not self._object_in_frame(object_cam):
                    fprint("Object not in frame")
                    continue
                
                # Get enu points associated with this totem and remove ones that are too low
                points_np = np.array(map(p2np, obj.points))
                height = np.max(points_np[:, 2]) - np.min(points_np[:, 2])
                if height < .1:
                    # If the height of the object is too small, skip (units are meters)
                    fprint("Object too small")
                    continue

                threshold = np.min(points_np[:, 2]) + self.height_remove * height  
                points_np = points_np[points_np[:, 2] > threshold]
                
                # Shove ones in there to make homogenous points to get points in image frame
                points_np_homo = np.hstack((points_np, np.ones((points_np.shape[0], 1)))).T
                points_cam = t_mat44.dot(points_np_homo).T
                points_px = map(self.camera_model.project3dToPixel, points_cam[:, :3])
                
                [cv2.circle(self.debug.image, tuple(map(int, p)), 2, (255, 255, 255), -1) for p in points_px]
                
                # Get color information from the points
                roi = self._get_ROI_from_points(points_px)
                h, s, v = self._get_hsv_from_ROI(roi, image)

                # Compute parameters that go into the confidence
                boat_q = self.odom[1]
                target_q = self._get_solar_angle()
                q_err = self._get_quaternion_error(boat_q, target_q)
                
                dist = np.linalg.norm(self.odom[0] - p2np(obj.position))

                fprint("H: {}, S: {}, V: {}".format(h, s, v))
                fprint("q_err: {}, dist: {}".format(q_err, dist))

                # Add to database and setup debug image
                if s < self.saturation_reject or v < self.value_reject:
                    err_msg = "The colors aren't expressive enough s: {} ({}) v: {} ({}). Rejecting."
                    fprint(err_msg.format(s, self.saturation_reject, v, self.value_reject), msg_color='red')

                else:
                    if obj.id not in self.colored:
                       self.colored[obj.id] = Observation() 
                    
                    # Add this observation in
                    self.colored[obj.id] += h, v, dist, q_err
                    print self.colored[obj.id]

                rgb = (0, 0, 0)
                color = self.do_estimate(obj.id)
                # Do we have a valid color estimate
                if color:
                    fprint("COLOR IS VALID", msg_color='green')
                    rgb = self.database_color_map[color[0]]
                    
                    cmd = '{name}={rgb[0]},{rgb[1]},{rgb[2]},{_id}'
                    self.make_request(cmd=cmd.format(name=obj.name,_id=obj.id, rgb=rgb))

                bgr = rgb[::-1]
                cv2.circle(self.debug.image, tuple(object_px), 10, bgr, -1)
                font = cv2.FONT_HERSHEY_SIMPLEX
                cv2.putText(self.debug.image, str(obj.id), tuple(object_px), font, 1, bgr, 2)

    def _get_solar_angle(self):
        return [0, 0, 0, 1]

    def _get_ROI_from_points(self, image_points):
        cnt = np.array([image_points]).astype(np.float32)
        rect = cv2.minAreaRect(cnt)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        fprint("Drawing rectangle")
        cv2.drawContours(self.debug.image, [box], 0, (0, 0, 255), 2)
        return box

    def _get_hsv_from_ROI(self, roi, img):
        mask = np.zeros(img.shape[:2], np.uint8)
        cv2.drawContours(mask, [roi], 0, 255, -1)
        bgr = cv2.mean(img, mask)
        # We have to treat that bgr value as a 3d array to get cvtColor to work
        bgr = np.array([[bgr]])[:, :3].astype(np.uint8)
        h, s, v = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)[0, 0]

        return h, s, v
        
        # Now check that s and v are in a good range
        if s < self.saturation_reject or v < self.value_reject:
            err_msg = "The colors aren't expressive enough s: {} ({}) v: {} ({}). Rejecting."
            fprint(err_msg.format(s, self.saturation_reject, v, self.value_reject), msg_color='red')
            return None

        # Compute hue error in SO2
        hue_angle = np.radians(h * 2) 
        
        # Display the current values
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(self.debug.image, "h_val: {}".format(np.degrees(hue_angle)), 
                    tuple(roi[1]), font, 1, (255, 255, 255), 2)

        likely_color, error = self.get_closest_color(hue_angle)

        if error > self.hue_error_reject:
            fprint("Closest color was {} with an error of {} rads (> {}). Rejecting.".format(likely_color, np.round(error, 3), 
                                                                                            self.hue_error_reject), msg_color='red')
            return None

        fprint("Likely color: {} with an hue error of {} rads.".format(likely_color, np.round(error, 3)))
        return [likely_color, error]

    def _object_in_frame(self, object_point):
        """
        Returns if the object is in frame given the centroid of the object.
        `object_point` should be in the camera_model's frame.
        """
        if object_point[2] < 0:
            return False

        px = np.array(self.camera_model.project3dToPixel(object_point))
        resoultion = self.camera_model.fullResolution()
        return not (np.any([0, 0] > px) or np.any(px > resoultion))


if __name__ == "__main__":
    rospy.init_node("database_colorer")
    c = Colorama()
    rospy.spin()
