#!/usr/bin/env python

from sub8_perception.srv import TorpBoardPoseRequest
import rospy

class TBPoseEstimator:
    def __init__(self):
        self.pose_est_service = rospy.Service('/torpedo_board/pose_est_srv', TorpBoardPoseRequest, self.optimize_pose)
        
    def optimize_pose(self, req):
        print 'I will optimize the pose'

if __name__ == '__main__':
    print '\x1b[1;31mInitializing the Torpedo Board Pose Estimation node\x1b[0m'
    pose_estimator = TBPoseEstimator()
    rospy.init_node('torpedo_board_pose_est')

