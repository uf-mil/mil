#!/usr/bin/python
from __future__ import division

import rospy
import cv2
import numpy as np
import time
import tf

from geometry_msgs.msg import PointStamped, PoseStamped
from navigator_msgs.srv import StartGate, StartGateResponse
import navigator_tools
import image_geometry


class ImageGetter(object):
    def __init__(self, topic_name):
        self.sub = navigator_tools.Image_Subscriber(topic_name, self.get_image)

        print 'getting topic', topic_name
        self.frame = None
        self.done = False

        self.sub.wait_for_camera_info()
        self.camera_info = self.sub.camera_info

    def get_image(self, msg):
        self.frame = msg
        self.done = True


class BuoySelector(object):
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


def intersect(A, a, B, b):
    # Based on http://morroworks.com/Content/Docs/Rays%20closest%20point.pdf
    # Finds the intersection of two rays `a` and `b` whose origin are `A` and `B`
    return (A + a * (-np.dot(a, b) * np.dot(b, B - A) + np.dot(a, B - A) * np.dot(b, b)) /
           (np.dot(a, a) * np.dot(b, b) - np.dot(a, b) ** 2) +
           B + b * (np.dot(a, b) * np.dot(a, B - A) - np.dot(b, B - A) * np.dot(a, a)) /
           (np.dot(a, a) * np.dot(b, b) - np.dot(a, b) ** 2)) / 2

def do_the_magic(pt1, pt2, cam_tf):
    '''
    pt1 is in the left frame and pt2 is in the right frame
    '''
    global left_cam, right_cam

    # I tried using a stereo camera model, but that wasn't working for some reason
    ray = np.linalg.inv(left_cam.K).dot(np.append(pt1, 1).reshape(-1))
    ray /= np.linalg.norm(ray)

    ray_1 = np.array(left_cam.projectPixelTo3dRay((pt1)))
    ray_2 = np.array(right_cam.projectPixelTo3dRay((pt2)))

    # I'm doing all the math in the camera frame
    origin_1 = np.array([0, 0, 0])
    origin_2 = np.array(cam_tf[0])  # Not accounting for different rotations
    point_np = intersect(origin_1, ray_1, origin_2, ray_2)

    return point_np

def do_buoys(srv, left, right, tf_listener):
    '''
    FYI:
        `left`: the left camera ImageGetter object
        `right`: the right camera ImageGetter object
    '''
    left_point = None
    right_point = None

    while not rospy.is_shutdown():
        # Get all the required TF links
        try:
            # Working copy of the current frame obtained at the same time as the tf link
            left_image, right_image = left.frame, right.frame

            cam_tf = tf_listener.lookupTransform(left.camera_info.header.frame_id.split('/')[-1],
                                                 right.camera_info.header.frame_id.split('/')[-1],
                                                 right.sub.last_image_time)
            cam_p, cam_q = tf_listener.lookupTransform("/enu",
                                                       left.camera_info.header.frame_id.split('/')[-1],
                                                       right.sub.last_image_time)
            cam_p = np.array([cam_p])
            cam_r = tf.transformations.quaternion_matrix(cam_q)[:3, :3]
            break
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException, TypeError) as e:
            print e
            rospy.logwarn("TF link not found.")
            time.sleep(.5)
            continue


    # Pick the red buoy out
    _left = BuoySelector("left", left_image, color=(10, 30, 250))
    red_left_pt, radius = _left.segment()
    if radius is None:
        return StartGateResponse(success=False)
    _right = BuoySelector("right", right_image, color=(10, 30, 250), draw_radius=radius)
    red_right_pt, radius = _right.segment()
    if radius is None:
        return StartGateResponse(success=False)

    # Pick the red buoy out
    _left = BuoySelector("left", left_image, color=(10, 250, 30))
    green_left_pt, radius = _left.segment()
    if radius is None:
        return StartGateResponse(success=False)
    _right = BuoySelector("right", right_image, color=(10, 250, 30), draw_radius=radius)
    green_right_pt, radius = _right.segment()
    if radius is None:
        return StartGateResponse(success=False)

    red_point_np = do_the_magic(red_left_pt, red_right_pt, cam_tf)
    red_point_np = np.array(cam_r.dot(red_point_np) + cam_p)[0]
    green_point_np = do_the_magic(green_left_pt, green_right_pt, cam_tf)
    green_point_np = np.array(cam_r.dot(green_point_np) + cam_p)[0]

    # Just for visualization
    for i in range(5):
        # Publish it 5 times so we can see it in rviz
        navigator_tools.draw_ray_3d(red_left_pt, left_cam, [1, 0, 0, 1],  m_id=0, frame="left")
        navigator_tools.draw_ray_3d(red_right_pt, right_cam, [1, 0, 0, 1],  m_id=1, frame="right")
        navigator_tools.draw_ray_3d(green_left_pt, left_cam, [0, 1, 0, 1],  m_id=2, frame="left")
        navigator_tools.draw_ray_3d(green_right_pt, right_cam, [0, 1, 0, 1],  m_id=3, frame="right")

        red_point = PointStamped()
        red_point.header = navigator_tools.make_header(frame="enu")
        red_point.point = navigator_tools.numpy_to_point(red_point_np)
        red_pub.publish(red_point)

        green_point = PointStamped()
        green_point.header = navigator_tools.make_header(frame="enu")
        green_point.point = navigator_tools.numpy_to_point(green_point_np)
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
    target_pose.header = navigator_tools.make_header(frame="enu")
    target_pose.pose = navigator_tools.numpy_quat_pair_to_pose(p, q)

    return StartGateResponse(target=target_pose, success=True)

if __name__ == "__main__":
    rospy.init_node("start_gate_manual_corresponder")
    tf_listener = tf.TransformListener()

    # For visualization
    red_pub = rospy.Publisher("red_buoy", PointStamped, queue_size=5)
    green_pub = rospy.Publisher("green_buoy", PointStamped, queue_size=5)


    left = ImageGetter('/front/left/image_raw')
    left_cam = image_geometry.PinholeCameraModel()
    left_cam.fromCameraInfo(left.camera_info)

    right = ImageGetter('/front/right/image_raw')
    right_cam = image_geometry.PinholeCameraModel()
    right_cam.fromCameraInfo(right.camera_info)

    while left.frame is None:
        print "Waiting for frame..."
        rospy.sleep(.5)

    s = rospy.Service("/vision/start_gate_buoys", StartGate,
                  lambda srv: do_buoys(srv, left, right, tf_listener))
    rospy.spin()
