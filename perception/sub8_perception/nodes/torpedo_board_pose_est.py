#!/usr/bin/env python
from sub8_msgs.srv import TorpBoardPoseRequest
import rospy
import numpy as np
import numpy.linalg as lin
from scipy import optimize
import tf
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion
from std_msgs.msg import ColorRGBA
import sub8_ros_tools as sub8_utils


class PoseObserver(object):
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0
        self.yaw = 0
        self.max_hist_size = 40
        self.xList = []
        self.yList = []
        self.zList = []
        self.yawList = []

    def push_back(self, pose):
        self.xList.append(pose[0])
        self.yList.append(pose[1])
        self.zList.append(pose[2])
        self.yawList.append(pose[3])
        if(len(self.xList) == self.max_hist_size + 1):
            self.xList.pop()
            self.yList.pop()
            self.zList.pop()
            self.yawList.pop()

    def get_pose_est_msg(self):
        ignore_num = int(len(self.xList) * 0.1)
        xInliers = sorted(self.xList)[ignore_num : -ignore_num]
        yInliers = sorted(self.yList)[ignore_num : -ignore_num]
        zInliers = sorted(self.zList)[ignore_num : -ignore_num]
        yawInliers = sorted(self.yawList)[ignore_num : -ignore_num]
        pose = Pose()
        if(len(xInliers) == 0): 
            return pose
        pose.position.x = sum(xInliers) / float(len(xInliers))
        pose.position.y = sum(yInliers) / float(len(yInliers))
        pose.position.z = sum(zInliers) / float(len(zInliers))
        yaw = sum(yawInliers) / float(len(yawInliers))
        quat = tf.transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation = Quaternion(*quat)
        print "\x1b[37m", pose, "\x1b[0m"
        return pose


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
        self.pose_obs = PoseObserver()
        self.pose_est_service = rospy.Service('/torpedo_board/pose_est_srv', TorpBoardPoseRequest, self.handle_pose_est_requests)
        self.tf_listener = tf.TransformListener()
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.pose_pub =  rospy.Publisher('/torpedo_board/pose', Pose, queue_size=100)
        self.marker_pub = rospy.Publisher('/torpedo_board/visualization/pose_est', visualization_msgs.Marker, queue_size=100)

    def handle_pose_est_requests(self, req):
        self.current_req = ParsedPoseEstRequest(req)
        # print self.current_req
        self.minimize_reprojection_error()
        print "Optimization success: ", self.minimization_result.success
        return self.minimization_result.success

    def minimize_reprojection_error(self):
        opt = {'disp': False}
        # Nelder-Mead and Powell seem to be the best methods for this problem
        self.minimization_result = optimize.minimize(self.calc_reprojection_error, self.current_req.pose, method='Nelder-Mead', options=opt)
        if self.minimization_result.success:
            if not rospy.is_shutdown():
                try:
                    self.tf_listener.waitForTransform('/map', 'stereo_front', self.current_req.stamp, rospy.Duration(0.1))
                    (trans, rot) = self.tf_listener.lookupTransform('/map', 'stereo_front', self.current_req.stamp)
                    yaw = tf.transformations.euler_from_quaternion(rot, 'syxz')[0]
                    pose = np.array([trans.x, trans.y, trans.z, yaw])
                    self.pose_obs.push_back(pose)
                except tf.Exception, e:
                    print "Exception! " + str(e)
                finally:
                    self.pose_pub.publish(self.pose_obs.get_pose_est_msg())
                    self.visualize_pose_est()
        else:
            print "\x1b[31mcost: " + str(self.minimization_result.fun) + "\x1b[0m"

    def calc_reprojection_error(self, pose_tb_from_cam):
        # pose_tb_from_cam = np.array([x, y, z, yaw])
        corners_from_cam_hom = self.generate_hom_tb_corners_from_cam(pose_tb_from_cam)
        L_cam_corners_hom = np.dot(self.current_req.left_cam_matx, corners_from_cam_hom)
        R_cam_corners_hom = np.dot(self.current_req.left_cam_matx, corners_from_cam_hom)
        L_cam_corners = np.divide(L_cam_corners_hom[0:2, :], L_cam_corners_hom[2])
        R_cam_corners = np.divide(R_cam_corners_hom[0:2, :], R_cam_corners_hom[2])
        reprojection_error = 0.0
        for i in xrange(4):
            reprojection_error = (reprojection_error
                                  + lin.norm(self.current_req.l_obs_corners[:, i] - L_cam_corners[:, i]))
            reprojection_error = (reprojection_error
                                  + lin.norm(self.current_req.r_obs_corners[:, i] - R_cam_corners[:, i]))
        return reprojection_error

    def generate_hom_tb_corners_from_cam(self, pose_tb_from_cam):
        # pose_tb_from_cam --> np.array([x, y, z, yaw])
        rot_tb_from_cam = tf.transformations.rotation_matrix(pose_tb_from_cam[3], np.array([[0], [1], [0]]))[:3, :3]
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

    def visualize_pose_est(self):
        marker = visualization_msgs.Marker(
            ns='torpedo_board/pose_est',
            id=0,
            header=sub8_utils.make_header(frame=self.current_req.frame_id),
            type=visualization_msgs.Marker.LINE_STRIP,
            action=visualization_msgs.Marker.ADD,
            color=ColorRGBA(0.0, 1.0, 10, 0.7),
            scale=Vector3(0.05, 0.0, 0.0),
            lifetime=rospy.Duration()
        )
        pts = self.generate_hom_tb_corners_from_cam(self.minimization_result.x)
        marker.points.append(Point(x=pts[0, 0], y=pts[1, 0], z=pts[2, 0]))
        marker.points.append(Point(x=pts[0, 1], y=pts[1, 1], z=pts[2, 1]))
        marker.points.append(Point(x=pts[0, 2], y=pts[1, 2], z=pts[2, 2]))
        marker.points.append(Point(x=pts[0, 3], y=pts[1, 3], z=pts[2, 3]))
        marker.points.append(Point(x=pts[0, 0], y=pts[1, 0], z=pts[2, 0]))
        self.marker_pub.publish(marker)


class ParsedPoseEstRequest(object):
    def __init__(self, req):
        self.seq = req.pose_stamped.header.seq
        self.stamp = rospy.Time.from_sec(req.pose_stamped.header.stamp.to_time())
        self.frame_id = req.pose_stamped.header.frame_id
        self.position = req.pose_stamped.pose.position
        self.orientation = req.pose_stamped.pose.orientation
        quat = np.array([self.orientation.x, self.orientation.y, self.orientation.z, self.orientation.w])
        self.pose = np.array([self.position.x, self.position.y, self.position.z,
                              tf.transformations.euler_from_quaternion(quat, 'syxz')[0]])
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
                   + "Orientation:\n" + w_on + str(self.orientation) + reset
                   + "Left camera projection matrix:\n" + w_on + str(self.left_cam_matx) + reset
                   + "Right camera projection matrix:\n" + w_on + str(self.right_cam_matx) + reset
                   + "Observed Board corners left image:\n" + w_on + str(self.l_obs_corners) + reset
                   + "Observed Board corners right image:\n" + w_on + str(self.r_obs_corners) + reset)
        return str_rep

# def tf_to_world_frame()


if __name__ == '__main__':
    print '\x1b[1;31mInitializing the Torpedo Board Pose Estimation node\x1b[0m'
    print "Awaiting pose estimation requests through:\n\t/torpedo_board_pose_est_srv\n"
    rospy.init_node('torpedo_board_pose_est')
    pose_estimator = TBPoseEstimator()
    rospy.spin()
