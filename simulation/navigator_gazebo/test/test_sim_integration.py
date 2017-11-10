#!/usr/bin/env python
import rospy
import unittest
import itertools
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

PKG = "navigator_gazebo"


class TestSimIntegration(unittest.TestCase):

    def __init__(self, *args):
        super(TestSimIntegration, self).__init__(*args)
        rospy.sleep(3.5)
        # subscribe to odometry topics
        self.odom_pos_msg = []
        self.odom_ori_msg = []
        rospy.Subscriber("/odom", Odometry, self.odom_cb)
        self.absodom_pos_msg = []
        self.absodom_ori_msg = []
        rospy.Subscriber("/absodom", Odometry, self.absodom_cb)
        # subscribe to camera info topics
        self.cam_info_msg = []
        self.cam_image_msg = []
        topics_cam_info = [
            "/camera/front/right/camera_info",
            "/camera/front/left/camera_info",
            "/camera/down/camera_info",
            "/camera/starboard/camera_info"]
        info_functions = [
            self.cam_info_right_cb,
            self.cam_info_left_cb,
            self.cam_info_down_cb,
            self.cam_info_starboard_cb]
        for topic, function in itertools.izip(topics_cam_info, info_functions):
            rospy.Subscriber(topic, CameraInfo, function)
        # subscribe to camera image topics
        self.image_msg = []
        topics_image = [
            "/camera/front/right/image_color",
            "/camera/front/left/image_color",
            "/camera/down/image_color",
            "/camera/starboard/image_color"]
        image_functions = [
            self.cam_image_right_cb,
            self.cam_image_left_cb,
            self.cam_image_down_cb,
            self.cam_image_starboard_cb]
        for topic, function in itertools.izip(topics_image, image_functions):
            rospy.Subscriber(topic, Image, function)
        # subscribe to pointcloud topic
        self.pc_info_msg = []
        self.pc_msg = []
        rospy.Subscriber("/velodyne_points", PointCloud2, self.points_cb)

    def odom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.odom_pos_msg = [pos.x, pos.y, pos.z]
        self.odom_ori_msg = [ori.x, ori.y, ori.z, ori.w]

    def test_odom(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while ((len(self.odom_pos_msg) < 3 or len(self.odom_ori_msg) < 4) and
                rospy.Time.now() < timeout):
            rospy.sleep(0.1)
        self.assertTrue(len(self.odom_pos_msg) == 3 and len(self.odom_ori_msg) == 4,
                        msg="POS, ORI: {}, {}".format(len(self.odom_pos_msg), len(self.odom_ori_msg)))
        initial_pos = [-1.2319, 0.0, 0.0]
        initial_ori = [0.0, 0.0, 0.0, 1.0]
        self.verify_pos_ori(
            self.odom_pos_msg,
            initial_pos,
            self.odom_ori_msg,
            initial_ori,
            "/odom")

    def absodom_cb(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        self.absodom_pos_msg = [pos.x, pos.y, pos.z]
        self.absodom_ori_msg = [ori.x, ori.y, ori.z, ori.w]

    def test_absodom(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while ((len(self.absodom_pos_msg) == 0 or len(self.absodom_ori_msg) ==
                0) and rospy.Time.now() < timeout):
            rospy.sleep(0.1)
        self.assertTrue(len(self.absodom_pos_msg) == 3 and len(self.absodom_ori_msg) == 4,
                        msg="POS, ORI: {}, {}".format(len(self.absodom_pos_msg), len(self.absodom_ori_msg)))
        initial_pos = [743789.637462, -5503821.36715, 3125622.10477]
        initial_ori = [0.0, 0.0, 0.0, 1.0]
        self.verify_pos_ori(
            self.absodom_pos_msg,
            initial_pos,
            self.absodom_ori_msg,
            initial_ori,
            "/absodom")

    def verify_pos_ori(self, pos, initial_pos, ori, initial_ori, topic):
        # make assertions
        for actual, initial in itertools.izip(pos, initial_pos):
            self.assertAlmostEqual(
                actual, initial, places=0, msg=(
                    "Error: {} position is: {} should be {}".format(
                        topic, actual, initial)))
        for actual, initial in itertools.izip(ori, initial_ori):
            self.assertEqual(
                actual, initial, msg=(
                    "Error: {} orientation is: {} should be {}".format(
                        topic, actual, initial)))

    def cam_info_right_cb(self, msg):
        if len(self.cam_info_msg) == 0:
            self.cam_info_msg.append([msg.width, msg.height])
        elif len(self.cam_info_msg) > 0:
            self.cam_info_msg[0] = [msg.width, msg.height]

    def cam_info_left_cb(self, msg):
        if len(self.cam_info_msg) == 1:
            self.cam_info_msg.append([msg.width, msg.height])
        elif len(self.cam_info_msg) > 1:
            self.cam_info_msg[1] = [msg.width, msg.height]

    def cam_info_down_cb(self, msg):
        if len(self.cam_info_msg) == 2:
            self.cam_info_msg.append([msg.width, msg.height])
        elif len(self.cam_info_msg) > 2:
            self.cam_info_msg[2] = [msg.width, msg.height]

    def cam_info_starboard_cb(self, msg):
        if len(self.cam_info_msg) == 3:
            self.cam_info_msg.append([msg.width, msg.height])
        elif len(self.cam_info_msg) > 3:
            self.cam_info_msg[3] = [msg.width, msg.height]

    def cam_image_right_cb(self, msg):
        if len(self.cam_image_msg) == 0:
            self.cam_image_msg.append(msg.data)
        elif len(self.cam_image_msg) > 0:
            self.cam_image_msg[0] = msg.data

    def cam_image_left_cb(self, msg):
        if len(self.cam_image_msg) == 1:
            self.cam_image_msg.append(msg.data)
        elif len(self.cam_image_msg) > 1:
            self.cam_image_msg[1] = msg.data

    def cam_image_down_cb(self, msg):
        if len(self.cam_image_msg) == 2:
            self.cam_image_msg.append(msg.data)
        elif len(self.cam_image_msg) > 2:
            self.cam_image_msg[2] = msg.data

    def cam_image_starboard_cb(self, msg):
        if len(self.cam_image_msg) == 3:
            self.cam_image_msg.append(msg.data)
        elif len(self.cam_image_msg) > 3:
            self.cam_image_msg[3] = msg.data

    def test_image(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while ((len(self.cam_info_msg) != 4 or len(self.cam_image_msg) !=
                4) and rospy.Time.now() < timeout):
            rospy.sleep(0.01)
        self.assertTrue(len(self.cam_info_msg) == 4 and len(self.cam_image_msg) ==
                        4, msg="{} {}".format(len(self.cam_info_msg), len(self.cam_image_msg)))
        # 960, 600
        initial_res = [0, 0]
        self.verify_info(self.cam_info_msg, initial_res, "/camera")
        self.verify_not_empty(self.cam_image_msg, 4)

    def points_cb(self, msg):
        self.pc_info_msg = [[msg.width, msg.height]]
        self.pc_msg = [msg.data]

    def test_pointcloud(self):
        timeout = rospy.Time.now() + rospy.Duration(1)
        while((len(self.pc_info_msg) != 1 or len(self.pc_msg) != 1) and rospy.Time.now() < timeout):
            rospy.sleep(0.01)
        self.assertTrue(len(self.pc_info_msg) == 1 and len(
            self.pc_msg) == 1,
            msg="INFO, DATA: {}, {}".format(len(self.pc_info_msg), len(self.pc_msg)))
        # 209, 1
        initial_res = [0, 0]
        self.verify_info(self.pc_info_msg, initial_res, "/velodyne_points")
        self.verify_not_empty(self.pc_msg, 1)

    def verify_not_empty(self, data_lists, num_topics):
        self.assertEqual(
            len(data_lists),
            num_topics,
            msg="Number of topics is {}, should be {}".format(
                len(data_lists),
                num_topics))
        for data_list in data_lists:
            self.assertNotEqual(len(data_list), 0, msg="data is empty")

    def verify_info(self, res_info, initial_info, topic):
        for msg in res_info:
            for actual_dim, initial_dim in itertools.izip(msg, initial_info):
                self.assertNotEqual(
                    actual_dim, initial_dim, msg=(
                        "Error: {} is: {} shouldn't be {}".format(
                            topic, actual_dim, initial_dim)))


if __name__ == '__main__':
    import rostest
    rospy.init_node('test_sim_integration', anonymous=True)
    print "name", rospy.get_name()
    print "namespace", rospy.get_namespace()
    rostest.rosrun(PKG, 'sim_integration_test', TestSimIntegration)
