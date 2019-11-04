#!/usr/bin/env python
from __future__ import division

from gazebo_msgs.srv import GetModelState
import geometry_msgs.msg
from mil_ros_tools import msg_helpers
import numpy as np
import rospy
from std_msgs.msg import Header
import tf
import tf2_ros

SUB_MODEL_NAME = 'sub8'
PINGER_MODEL_NAME = 'red'  # Pinger model not in Gazebo yet TODO!


class PingerTFPublisher():

    def __init__(self, pinger_model_name):
        '''
        @type pinger_model_name: string
        @param pinger_model_name: Name of the pinger gazebo model
        '''
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state',
                                            GetModelState)
        self.pinger_model_name = pinger_model_name
        self.br = tf2_ros.TransformBroadcaster()
        self.tf = geometry_msgs.msg.TransformStamped()
        self.tf.child_frame_id = 'pinger'
        self.seq = 0
        self.timer = rospy.Timer(rospy.Duration(0.02), self.publish_tf)

    def publish_tf(self, *args):
        ''' Publishes a tf transform from base_link to pinger frame
        The orientation of the pinger frame is arbitrary

        Note there should be a more efficient way of getting this transform
        into the tf tree, and ideally it would be a child of /map, not
        /base_link. However, this works so it's ok for now
        '''
        sub_pose = msg_helpers.pose_to_numpy(
            self.get_model(model_name=SUB_MODEL_NAME).pose)
        if not hasattr(self, 'pinger_pose'):
            self.pinger_pose = msg_helpers.pose_to_numpy(
                self.get_model(model_name=self.pinger_model_name).pose)

        sub_T, sub_R = sub_pose
        pinger_T, pinger_R = self.pinger_pose

        sub_R = tf.transformations.quaternion_matrix(sub_R)[:3, :3]
        pinger_R = tf.transformations.quaternion_matrix(pinger_R)[:3, :3]
        sub_pinger_T = np.linalg.inv(sub_R).dot(pinger_T - sub_T)
        sub_pinger_R = pinger_R

        sub_pinger_pose = msg_helpers.numpy_pair_to_pose(sub_pinger_T, sub_pinger_R)
        self.tf.transform.translation = sub_pinger_pose.position
        self.tf.transform.rotation = sub_pinger_pose.orientation
        self.tf.header = Header(seq=self.seq, stamp=rospy.Time.now(), frame_id='base_link')
        self.seq = self.seq + 1
        self.br.sendTransform(self.tf)


if __name__ == '__main__':
    rospy.init_node('pinger_tf_pub')
    PingerTFPublisher(PINGER_MODEL_NAME)
    rospy.spin()
