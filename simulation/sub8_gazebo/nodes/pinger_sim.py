#!/usr/bin/env python
import rospy
from gazebo_msgs.srv import GetModelState
from sub8_msgs.srv import Sonar
from sub8_sonar import EchoLocator
from sub8_ros_tools import msg_helpers
from sub8_ros_tools.geometry_helpers import rotate_vect_by_quat
import tf

import numpy as np


class Pinger():
    def __init__(self, wave_propagation_speed, precision=14):
        self.get_model = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)

        self.wave_propagation_speed = wave_propagation_speed
        self.precision = precision

        self.hydrophone_board_tf = np.array([0, 0, 0])
        self.hydro_loc =  {'hydro0': {'x': 0,'y': 0,'z': 0},
                           'hydro1': {'x': -0.0254, 'y': 0, 'z': 0.0254},
                           'hydro2': {'x': 0.0254, 'y': 0, 'z': 0},
                           'hydro3': {'x': 0, 'y': -0.0254, 'z': 0} }

        self.echo_locator = EchoLocator(self.hydro_loc, wave_propagation_speed)

        # Wait for all the models and such to spawn.
        rospy.Service('/sonar/get_pinger_pulse', Sonar, self.request_data)

        rospy.sleep(3)
        print self.request_data(None)

    def request_data(self, srv):
        '''
        Calculate timestamps based on the subs position and the pinger location then use the sonar driver's
            EchoLocator to estimate the position.
        '''

        sub_state = self.get_model(model_name='sub8')
        pinger_state = self.get_model(model_name='pinger')

        # Convert full state to poses
        sub_pose = msg_helpers.pose_to_numpy(sub_state.pose)
        hydro_board_pose = sub_pose[0] + self.hydrophone_board_tf
        pinger_pose = msg_helpers.pose_to_numpy(pinger_state.pose)[0]

        print sub_pose[1]
        # Calculate distances to each hydrophone from the pinger (need to be accounting for sub rotation).
        hydro_poses = np.array([[self.hydro_loc['hydro0']['x'], self.hydro_loc['hydro0']['y'], self.hydro_loc['hydro0']['z']],
                                [self.hydro_loc['hydro1']['x'], self.hydro_loc['hydro1']['y'], self.hydro_loc['hydro1']['z']],
                                [self.hydro_loc['hydro2']['x'], self.hydro_loc['hydro2']['y'], self.hydro_loc['hydro2']['z']],
                                [self.hydro_loc['hydro3']['x'], self.hydro_loc['hydro3']['y'], self.hydro_loc['hydro3']['z']]]) + hydro_board_pose
        print hydro_poses
        distances = np.linalg.norm(pinger_pose - hydro_poses, axis=1) 
        timestamps = np.round(distances / self.wave_propagation_speed, decimals=self.precision)
        print timestamps - timestamps[0]

        # Don't forget estimated location is in map frame, transform to sub frame.
        est_location = self.echo_locator.getPulseLocation(timestamps - timestamps[0])
        est_location_np = rotate_vect_by_quat(np.array([est_location.x, est_location.y, est_location.z, 0]) - np.hstack((sub_pose[0], 0)), sub_pose[1])
        est_location.x = est_location_np[0]
        est_location.y = est_location_np[1]
        est_location.z = est_location_np[2]

        return est_location

if __name__ == '__main__':
    rospy.init_node('turbulator')
    Pinger(1482)
    rospy.spin()
