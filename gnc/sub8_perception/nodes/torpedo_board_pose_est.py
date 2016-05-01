#!/usr/bin/env python

from sub8_perception.srv import TorpBoardPoseRequest
import rospy
import numpy as np

# def parse_TB_pose_request(req):

class TBPoseEstimator(object):
    def __init__(self):
        self.pose_est_service = rospy.Service('/torpedo_board/pose_est_srv', TorpBoardPoseRequest, self.optimize_pose)
        
    def handle_pose_est_requests(self,req):
        print pose_est_request
        pose_est_request = ParsedPoseEstRequest(req)

    def optimize_pose(self, req):
        #TODO: scipy.optimize stuff

class ParsedPoseEstRequest(object):
 	def __init__(self, req):
 		self.seq = req.pose_stamped.header.seq
 		self.stamp = req.pose_stamped.header.stamp
 		self.frame_id = req.pose_stamped.header.frame_id
 		self.position = req.pose_stamped.pose.position
 		self.orientation = req.pose_stamped.pose.orientation
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
 			[req.l_obs_corners[3].x, req.l_obs_corners[3].y]])
 		self.r_obs_corners = np.array([
 			[req.r_obs_corners[0].x, req.r_obs_corners[0].y],
 			[req.r_obs_corners[1].x, req.r_obs_corners[1].y],
 			[req.r_obs_corners[2].x, req.r_obs_corners[2].y],
 			[req.r_obs_corners[3].x, req.r_obs_corners[3].y]])

 	def __str__(self):
 		# To get rid of color characters: w_on = ""; reset = "\n"
 		w_on = "\x1b[37m"
 		reset = "\x1b[0m\n"
 		str_rep = ("Pose Estimation request:\n"
 			+ "Seq: " + w_on + str(self.seq) + reset
 			+ "Stamp: " + w_on + str(self.stamp) + reset
 			+ "Frame_ID: " + w_on + self.frame_id + reset
 			+ "Position:\n" + w_on + str(self.position) + reset
 			+ "Orientation:\n" + w_on + str(self.orientation) + reset
 			+ "Left camera projection matrix:\n" + w_on + str(self.left_cam_matx) + reset
 			+ "Right camera projection matrix:\n" + w_on + str(self.right_cam_matx) + reset
 			+ "Observed Board corners left image:\n" + w_on + str(self.l_obs_corners) + reset
 			+ "Observed Board corners right image:\n" + w_on + str(self.r_obs_corners) + reset)
 		return str_rep


if __name__ == '__main__':
    print '\x1b[1;31mInitializing the Torpedo Board Pose Estimation node\x1b[0m'
    print "Awaiting pose estimation requests through:\n\t/torpedo_board_pose_est_srv"
    rospy.init_node('torpedo_board_pose_est')
    pose_estimator = TBPoseEstimator()
    rospy.spin()

