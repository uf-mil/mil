#!/usr/bin/env python

from sub8_perception.srv import TorpBoardPoseRequest
import rospy
import numpy as np
import numpy.linalg as lin
from scipy import optimize
import tf
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import sub8_ros_tools as sub8_utils


class TBPoseEstimator(object):
    def __init__(self):
        self.H = 1.24  # in meters
        self.W = 0.61  # in meters
        self.corners_from_tb_hom = np.array([
            [-self.W/2.0, -self.H/2.0, 0.0, 1.0],  # TL
            [ self.W/2.0, -self.H/2.0, 0.0, 1.0],  # TR
            [ self.W/2.0,  self.H/2.0, 0.0, 1.0],  # BR
            [-self.W/2.0,  self.H/2.0, 0.0, 1.0]], # BL
            dtype=np.float32).T
        self.pose_est_service = rospy.Service('/torpedo_board/pose_est_srv', TorpBoardPoseRequest, self.handle_pose_est_requests)
        self.marker_pub = rospy.Publisher('/torpedo_board/visualization/pose_est', visualization_msgs.Marker, queue_size=100)

    def handle_pose_est_requests(self, req):
        pose_est_request = ParsedPoseEstRequest(req)
        # print pose_est_request
        set_cam_mats(self, pose_est_request)  # runs once
        self.L_obs_corners = pose_est_request.l_obs_corners
        self.R_obs_corners = pose_est_request.r_obs_corners
        self.pose_req_frame = pose_est_request.frame_id
        x_0 = pose_est_request.position.x
        y_0 = pose_est_request.position.y
        z_0 = pose_est_request.position.z
        yaw_0 = pose_est_request.yaw
        # print pose_est_request.position
        # print yaw_0
        # init_guess = np.array([0, 0, 1.4, 0])
        init_guess = np.array([x_0, y_0, z_0, yaw_0])
        self.optimize_pose(init_guess)
        return self.optimization_success

    def generate_hom_tb_corners_from_cam(self, pose_tb_from_cam):
        # pose_tb_from_cam = np.array([x, y, z, yaw])
        rot_tb_from_cam = create_yaw_matrix(pose_tb_from_cam[3])
        trans_tb_from_cam = np.array([
            [pose_tb_from_cam[0]], [pose_tb_from_cam[1]], [-pose_tb_from_cam[2]]])
        # TODO: figure out why a negative is needed above, is there a problem with my frames?
        homogenizer = np.array([[0, 0, 0, 1]])
        tf_cam_from_tb_hom = np.vstack((
            np.hstack((
                rot_tb_from_cam.T,
                -np.dot(rot_tb_from_cam.T, trans_tb_from_cam))),
            homogenizer))
        corners_from_cam_hom = np.dot(tf_cam_from_tb_hom, self.corners_from_tb_hom)
        return corners_from_cam_hom

    def calc_reprojection_error(self, pose_tb_from_cam):
        # pose_tb_from_cam = np.array([x, y, z, yaw])
        corners_from_cam_hom = self.generate_hom_tb_corners_from_cam(pose_tb_from_cam)
        L_cam_corners_hom = np.dot(self.L_cam_mat, corners_from_cam_hom)
        R_cam_corners_hom = np.dot(self.R_cam_mat, corners_from_cam_hom)
        L_cam_corners = np.divide(L_cam_corners_hom[0:2, :], L_cam_corners_hom[2])
        R_cam_corners = np.divide(R_cam_corners_hom[0:2, :], R_cam_corners_hom[2])
        reprojection_error = 0.0
        for i in xrange(4):
            reprojection_error = (reprojection_error
                                  + lin.norm(self.L_obs_corners[:, i] - L_cam_corners[:, i]))
            reprojection_error = (reprojection_error
                                  + lin.norm(self.R_obs_corners[:, i] - R_cam_corners[:, i]))
        return reprojection_error

    def optimize_pose(self, init_guess):
        self.calc_reprojection_error(init_guess)
        opt = {'disp': False}
        result = optimize.minimize(self.calc_reprojection_error, init_guess, method='Nelder-Mead', options=opt)
        self.optimization_success = result.success
        if self.optimization_success:
            self.prelim_pose_est = result.x
            # print self.prelim_pose_est
            # print "cost: " + str(result.fun)
            self.visualize_pose_est()
        else:
            print "\x1b[31mcost: " + str(result.fun) + "\x1b[0m"
        # print prelim_pose_est

    def visualize_pose_est(self):
        pts = self.generate_hom_tb_corners_from_cam(self.prelim_pose_est)
        p1 = Point(x=pts[0, 0], y=pts[1, 0], z=pts[2, 0])
        p2 = Point(x=pts[0, 1], y=pts[1, 1], z=pts[2, 1])
        p3 = Point(x=pts[0, 2], y=pts[1, 2], z=pts[2, 2])
        p4 = Point(x=pts[0, 3], y=pts[1, 3], z=pts[2, 3])
        marker = visualization_msgs.Marker(
            ns='torpedo_board/pose_est',
            id=0,
            header=sub8_utils.make_header(frame=self.pose_req_frame),
            type=visualization_msgs.Marker.LINE_STRIP,
            action=visualization_msgs.Marker.ADD,
            color=ColorRGBA(0.0, 1.0, 10, 0.7),
            scale=Vector3(0.05, 0.0, 0.0),
            lifetime=rospy.Duration()
        )
        marker.points.append(p1)
        marker.points.append(p2)
        marker.points.append(p3)
        marker.points.append(p4)
        marker.points.append(p1)
        self.marker_pub.publish(marker)


class ParsedPoseEstRequest(object):
    def __init__(self, req):
        self.seq = req.pose_stamped.header.seq
        self.stamp = req.pose_stamped.header.stamp
        self.frame_id = req.pose_stamped.header.frame_id
        self.position = req.pose_stamped.pose.position
        self.orientation = req.pose_stamped.pose.orientation
        quat = np.array(
            [self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        self.yaw = tf.transformations.euler_from_quaternion(quat, 'syxz')[0]
        self.left_cam_matx = np.array([
            [req.l_proj_mat[0], req.l_proj_mat[1], req.l_proj_mat[2], req.l_proj_mat[3]],
            [req.l_proj_mat[4], req.l_proj_mat[5], req.l_proj_mat[6], req.l_proj_mat[7]],
            [req.l_proj_mat[8], req.l_proj_mat[9], req.l_proj_mat[10], req.l_proj_mat[11]]])
        self.right_cam_matx = np.array([
            [req.r_proj_mat[0], req.r_proj_mat[1], req.r_proj_mat[2], req.r_proj_mat[3]],
            [req.r_proj_mat[4], req.r_proj_mat[5], req.r_proj_mat[6], req.r_proj_mat[7]],
            [req.r_proj_mat[8], req.r_proj_mat[9], req.r_proj_mat[10], req.r_proj_mat[11]]])
        self.l_obs_corners = np.array([
            [req.l_obs_corners[0].x, req.l_obs_corners[0].y],
            [req.l_obs_corners[1].x, req.l_obs_corners[1].y],
            [req.l_obs_corners[2].x, req.l_obs_corners[2].y],
            [req.l_obs_corners[3].x, req.l_obs_corners[3].y]]).T
        self.r_obs_corners = np.array([
            [req.r_obs_corners[0].x, req.r_obs_corners[0].y],
            [req.r_obs_corners[1].x, req.r_obs_corners[1].y],
            [req.r_obs_corners[2].x, req.r_obs_corners[2].y],
            [req.r_obs_corners[3].x, req.r_obs_corners[3].y]]).T

    def __str__(self):
        # To get rid of color characters: w_on = ""; reset = "\n"
        w_on = "\x1b[37m"
        reset = "\x1b[0m\n"
        str_rep = ("Pose Estimation request:\n"
                   + "Seq: " + w_on + str(self.seq) + reset
                   + "Stamp: " + w_on + str(self.stamp) + reset
                   + "Frame_ID: " + w_on + self.frame_id + reset
                   + "Position:\n" + w_on + str(self.position) + reset
                   + "Yaw: " + w_on + str(self.yaw * 180.0 / 3.1459) + " deg" + reset
                   # + "Orientation:\n" + w_on + str(self.orientation) + reset
                   + "Left camera projection matrix:\n" + w_on + str(self.left_cam_matx) + reset
                   + "Right camera projection matrix:\n" + w_on + str(self.right_cam_matx) + reset
                   + "Observed Board corners left image:\n" + w_on + str(self.l_obs_corners) + reset
                   + "Observed Board corners right image:\n" + w_on + str(self.r_obs_corners) + reset)
        return str_rep


def set_cam_mats(tb_pose_est_obj, parsed_req):
    tb_pose_est_obj.L_cam_mat = parsed_req.left_cam_matx
    tb_pose_est_obj.R_cam_mat = parsed_req.right_cam_matx
    set_cam_mats.func_code = (lambda a, b: None).func_code  # Func should only run once


def create_yaw_matrix(yaw):
    c, s = np.cos(yaw), np.sin(yaw)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]], dtype=np.float32)


if __name__ == '__main__':
    print '\x1b[1;31mInitializing the Torpedo Board Pose Estimation node\x1b[0m'
    print "Awaiting pose estimation requests through:\n\t/torpedo_board_pose_est_srv"
    rospy.init_node('torpedo_board_pose_est')
    pose_estimator = TBPoseEstimator()
    rospy.spin()