#!/usr/bin/python
from __future__ import division

import rospy
import cv2
import numpy as np
import time
import tf

from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped, PoseStamped
from navigator_msgs.srv import StartGate, StartGateResponse
import mil_tools
import image_geometry
from scipy.optimize import minimize


class ImageGetter(object):
    def __init__(self, topic_name):
        self.sub = mil_tools.Image_Subscriber(topic_name, self.get_image)

        print 'getting topic', topic_name
        self.frame = None
        self.done = False

        self.camera_info = self.sub.wait_for_camera_info()

    def get_image(self, msg):
        self.frame = msg
        self.done = True


class BuoySelector(object):
    '''
    Allows user to manually click the buoys
    '''

    def __init__(self, name, img, scale_factor=1, color=(10, 75, 250), draw_radius=10):
        assert img is not None, "Image is none"

        cv2.namedWindow(name)
        cv2.setMouseCallback(name, self.mouse_cb)

        self.name = name
        self.point = None
        self.draw_radius = draw_radius
        self.color = color

        self.scale_factor = scale_factor
        self.img = cv2.resize(img, None, fx=scale_factor, fy=scale_factor, interpolation=cv2.INTER_CUBIC)
        self.draw_img = self.img

        self.mouse_position = (0, 0)

    def segment(self):
        while self.point is None:
            draw_img = np.copy(self.img)
            cv2.circle(draw_img, self.mouse_position, self.draw_radius, self.color, 1)
            cv2.imshow(self.name, draw_img)

            k = cv2.waitKey(10) & 0xFF

            if k == 27:  # Esc
                cv2.destroyAllWindows()
                rospy.sleep(.2)

                return None, None
            elif k == ord('q'):
                self.draw_radius += 2
            elif k == ord('a'):
                self.draw_radius -= 2

        cv2.destroyAllWindows()
        rospy.sleep(.2)
        pt = self.point / self.scale_factor
        self.point = None
        return pt, self.draw_radius

    def mouse_cb(self, event, x, y, flags, param):
        self.draw_img = np.copy(self.img)
        self.mouse_position = (x, y)

        if event == cv2.EVENT_LBUTTONUP:
            self.point = np.array([x, y])


class Segmenter(object):
    def __init__(self, color_space, ranges, invert):
        self.color_space = color_space
        self.lower = np.array(ranges['lower'])
        self.upper = np.array(ranges['upper'])
        self.invert = invert

    def segment(self, orig_image, debug_color=False):
        '''
        Make sure input image is BGR
        If you want only the debug image returned, pass a color in for `debug_color` (ex. (255, 0, 0) for blue)
        '''
        image = orig_image if self.color_space == 'bgr' else cv2.cvtColor(orig_image, cv2.COLOR_BGR2HSV)

        mask = cv2.inRange(image, self.lower, self.upper)
        filtered_mask = self.filter_mask(mask)

        cnts, _ = cv2.findContours(filtered_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cnt = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        M = cv2.moments(cnt)
        center = np.array([M['m10'], M['m01']]) / M['m00']

        if debug_color:
            debug_img = np.copy(orig_image)
            x, y, w, h = cv2.boundingRect(cnt)
            cv2.rectangle(debug_img, (x, y), (x + w, y + h), debug_color, 2)
            cv2.circle(debug_img, tuple(center.astype(np.int16)), 2, debug_color, -1)
            return debug_img

        return center, cv2.contourArea(cnt)

    def filter_image(self, image):
        filtered = cv2.GaussianBlur(image, (5, 5), 0)
        return filtered

    def filter_mask(self, mask):
        kernel = np.ones((5, 5), np.uint8)
        filtered = cv2.erode(mask, kernel, iterations=1)
        filtered = cv2.dilate(filtered, kernel, iterations=1)
        return filtered


def intersect(A, a, B, b):
    # Based on http://morroworks.com/Content/Docs/Rays%20closest%20point.pdf
    # Finds the intersection of two rays `a` and `b` whose origin are `A` and `B`
    return (A + a * (-np.dot(a, b) * np.dot(b, B - A) + np.dot(a, B - A) * np.dot(b, b)) /
            (np.dot(a, a) * np.dot(b, b) - np.dot(a, b) ** 2) +
            B + b * (np.dot(a, b) * np.dot(a, B - A) - np.dot(b, B - A) * np.dot(a, a)) /
            (np.dot(a, a) * np.dot(b, b) - np.dot(a, b) ** 2)) / 2


def minimize_repro_error(left_cam, right_cam, pt_l, pt_r, estimation):

    def f(x):
        left_error = np.linalg.norm(left_cam.project3dToPixel((x)) - pt_l)
        right_error = np.linalg.norm(right_cam.project3dToPixel((x)) - pt_r)
        return left_error ** 2 + right_error ** 2

    correction = minimize(
        fun=f,
        x0=estimation,
        tol=1e-9,
        method="TNC",
    )
    return correction.x


def do_the_magic(pt_l, pt_r, cam_tf):
    '''
    pt_l is in the left frame and pt_r is in the right frame
    '''
    global left_cam, right_cam

    ray_1 = np.array(left_cam.projectPixelTo3dRay((pt_l)))
    ray_2 = np.array(right_cam.projectPixelTo3dRay((pt_r)))

    # I'm doing all the math in the camera frame
    origin_1 = np.array([0, 0, 0])
    origin_2 = np.array(cam_tf[0])  # Not accounting for different rotations
    inital_estimate = intersect(origin_1, ray_1, origin_2, ray_2)

    # Now re-project points and find the minimum error
    estimate = minimize_repro_error(left_cam, right_cam, pt_l, pt_r, inital_estimate)

    return estimate


def load_from_parameter(color):
    param_name = '/start_gate/{}'.format(color)
    if not rospy.has_param(param_name):
        rospy.logerr("No parameters have been set!")
        rospy.signal_shutdown("Requires param to be set: {}".format(param_name))
        # exit()

    param = rospy.get_param(param_name)

    s = Segmenter(param['color_space'], param['ranges'], param['invert'])
    return s


def do_buoys(srv, left, right, red_seg, green_seg, tf_listener):
    '''
    FYI:
        `left`: the left camera ImageGetter object
        `right`: the right camera ImageGetter object
    '''

    while not rospy.is_shutdown():
        # Get all the required TF links
        try:
            # Working copy of the current frame obtained at the same time as the tf link
            tf_listener.waitForTransform("enu", "front_left_cam_optical", rospy.Time(), rospy.Duration(4.0))
            left_image, right_image = left.frame, right.frame
            cam_tf = tf_listener.lookupTransform("front_left_cam_optical", "front_right_cam_optical", left.sub.last_image_time)
            cam_p, cam_q = tf_listener.lookupTransform("enu", "front_left_cam_optical", left.sub.last_image_time)
            cam_p = np.array([cam_p])
            cam_r = tf.transformations.quaternion_matrix(cam_q)[:3, :3]
            break

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, TypeError) as e:
            print e
            rospy.logwarn("TF link not found.")
            time.sleep(.5)
            continue

    red_left_pt, rl_area = red_seg.segment(left_image)
    red_right_pt, rr_area = red_seg.segment(right_image)
    green_left_pt, gl_area = green_seg.segment(left_image)
    green_right_pt, gr_area = green_seg.segment(right_image)

    area_tol = 50
    if np.linalg.norm(rl_area - rr_area) > area_tol or np.linalg.norm(gl_area - gr_area) > area_tol:
        rospy.logwarn("Unsafe segmentation")
        StartGateResponse(success=False)

    red_point_np = do_the_magic(red_left_pt, red_right_pt, cam_tf)
    red_point_np = np.array(cam_r.dot(red_point_np) + cam_p)[0]
    green_point_np = do_the_magic(green_left_pt, green_right_pt, cam_tf)
    green_point_np = np.array(cam_r.dot(green_point_np) + cam_p)[0]

    # Just for visualization
    for i in range(5):
        # Publish it 5 times so we can see it in rviz
        mil_tools.draw_ray_3d(red_left_pt, left_cam, [1, 0, 0, 1], m_id=0, frame="front_left_cam_optical")
        mil_tools.draw_ray_3d(red_right_pt, right_cam, [1, 0, 0, 1], m_id=1, frame="front_right_cam_optical")
        mil_tools.draw_ray_3d(green_left_pt, left_cam, [0, 1, 0, 1], m_id=2, frame="front_left_cam_optical")
        mil_tools.draw_ray_3d(green_right_pt, right_cam, [0, 1, 0, 1], m_id=3, frame="front_right_cam_optical")

        red_point = PointStamped()
        red_point.header = mil_tools.make_header(frame="enu")
        red_point.point = mil_tools.numpy_to_point(red_point_np)
        red_pub.publish(red_point)

        green_point = PointStamped()
        green_point.header = mil_tools.make_header(frame="enu")
        green_point.point = mil_tools.numpy_to_point(green_point_np)
        green_pub.publish(green_point)

        time.sleep(1)

    # Now we have two points, find their midpoint and calculate a target angle
    midpoint = (red_point_np + green_point_np) / 2
    between_vector = green_point_np - red_point_np  # Red is on the left when we go out.
    yaw_theta = np.arctan2(between_vector[1], between_vector[0])
    # Rotate that theta by 90 deg to get the angle through the buoys
    yaw_theta += np.pi / 2.0

    p = midpoint
    q = tf.transformations.quaternion_from_euler(0, 0, yaw_theta)

    target_pose = PoseStamped()
    target_pose.header = mil_tools.make_header(frame="enu")
    target_pose.pose = mil_tools.numpy_quat_pair_to_pose(p, q)

    return StartGateResponse(target=target_pose, success=True)


def publish_debug(red_pub, green_pub, red_seg, green_seg, camera, *args):
    r_debug = red_seg.segment(camera.frame, (0, 10, 250))
    g_debug = green_seg.segment(camera.frame, (0, 250, 10))

    red_pub.publish(mil_tools.make_image_msg(r_debug))
    green_pub.publish(mil_tools.make_image_msg(g_debug))


if __name__ == "__main__":
    rospy.init_node("start_gate_perception")
    tf_listener = tf.TransformListener()

    # For visualization
    red_pub = rospy.Publisher("red_buoy", PointStamped, queue_size=5)
    green_pub = rospy.Publisher("green_buoy", PointStamped, queue_size=5)
    red_debug = rospy.Publisher("vision/start_gate/red", Image, queue_size=5)
    green_debug = rospy.Publisher("vision/start_gate/green", Image, queue_size=5)

    left = ImageGetter('/camera/front/left/image_rect_color')
    left_cam = image_geometry.PinholeCameraModel()
    left_cam.fromCameraInfo(left.camera_info)

    right = ImageGetter('/camera/front/right/image_rect_color')
    right_cam = image_geometry.PinholeCameraModel()
    right_cam.fromCameraInfo(right.camera_info)

    while left.frame is None and right.frame is None:
        print "Waiting for frame..."
        rospy.sleep(.5)

    red_seg = load_from_parameter("red")
    green_seg = load_from_parameter("green")

    # while not rospy.is_shutdown():
    #     l_center, debug_img = red.segment(left.frame, (0, 70, 255))
    #     red_debug.publish(mil_tools.make_image_msg(debug_img))
    #     rospy.sleep(1)

    s = rospy.Service("/vision/start_gate_buoys", StartGate,
                      lambda srv: do_buoys(srv, left, right, red_seg, green_seg, tf_listener))

    # rospy.Timer(rospy.Duration(.5), lambda republish: publish_debug(red_debug, green_debug,
    #                                                                 red_seg, green_seg, left))

    rospy.spin()
