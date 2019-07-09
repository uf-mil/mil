#!/usr/bin/env python
import numpy as np
import rospy
from urdf_parser_py.urdf import URDF
import tf2_ros
from tf.transformations import euler_from_quaternion
from mil_tools import rosmsg_to_numpy


class ThrusterMap(object):
    '''
    Helper class to map between global body forces / torques and thruster outputs, which are in a
    arbitrary effort unit. See thruster_mapper_node.py for usage example.
    '''

    def __init__(self, names, positions, angles, effort_ratio, effort_limit, com=np.zeros(2), joints=None):
        '''
        Creates a ThurserMapper instance
        param positions: list of X Y positions for each thuster [(x, y), (x,y), ...],
                         in order of ThurserMap.THRUSTERS, meters
        param angles: list of angles for each thruster [theta, theta, ...] in order of ThurserMap.THRUSTERS, radians
        param effort_ratio: linear mapping from force to effort units, so thrust = effort_ratio * force
        param effort_limit: maximum effort in either direction that should be commanded to a thruster
        param com: offset of boat's true center of mass to the frame thruster positions are given (base_link)
                   defaults to (0, 0, 0) incase we don't know com


        The mapping from body wrench to individual thrusts or visa versa is a simple least square solver.
        Bibliography:
          [1] Christiaan De Wit
              "Optimal Thrust Allocation Methods for Dynamic Positioning of Ships"
              see: http://repository.tudelft.nl/assets/uuid:4c9685ac-3f76-41c0-bae5-a2a96f4d757e/DP_Report_FINAL.pdf

        TODO:
          - dynamic reconfigure ???
          - implement a more interesting solver, which attempts to minimize energy output like SubjuGator
        '''
        self.names = names
        self.joints = joints
        self.effort_ratio = effort_ratio
        self.effort_limit = effort_limit

        ''' Iterate through thruster positions and create thruster trans matrix'''
        thruster_matrix = []
        # loop through all sub positions and compute collumns of A
        for thruster_number, position in enumerate(positions):
            # l_x and l_y are the offset for the center of gravity
            l_x, l_y = np.subtract(position, com)
            # sin and cos of the angle of the thrusters
            cos = np.cos(angles[thruster_number])
            sin = np.sin(angles[thruster_number])
            torque_effect = np.cross((l_x, l_y), (cos, sin))
            # must transpose to stack correctly
            thruster_column = np.transpose(np.array([[cos, sin, torque_effect]]))
            thruster_matrix.append(thruster_column)

        # returns a matrix made of the thruster collumns
        self.thruster_matrix = np.hstack(thruster_matrix)
        self.thruster_matrix_inv = np.linalg.pinv(self.thruster_matrix)  # Magical numpy psuedoinverse

    @classmethod
    def from_urdf(cls, urdf_string, transmission_suffix='_thruster_transmission'):
        '''
        Load from an URDF string. Expects each thruster to be connected a transmission ending in the specified suffix.
        A transform between the propeller joint and base_link must be available
        '''
        urdf = URDF.from_xml_string(urdf_string)
        buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buff)  # noqa
        names = []
        joints = []
        positions = []
        angles = []
        limit = -1
        ratio = -1
        for transmission in urdf.transmissions:
            find = transmission.name.find(transmission_suffix)
            if find != -1 and find + len(transmission_suffix) == len(transmission.name):
                if len(transmission.joints) != 1:
                    raise Exception('Transmission {} does not have 1 joint'.format(transmission.name))
                if len(transmission.actuators) != 1:
                    raise Exception('Transmission {} does not have 1 actuator'.format(transmission.name))
                t_ratio = transmission.actuators[0].mechanicalReduction
                if ratio != -1 and ratio != t_ratio:
                    raise Exception('Transmission {} has a different reduction ratio (not supported)'.format(t_ratio))
                ratio = t_ratio
                joint = None
                for t_joint in urdf.joints:
                    if t_joint.name == transmission.joints[0].name:
                        joint = t_joint
                if joint is None:
                    rospy.logerr('Transmission joint {} not found'.format(transmission.joints[0].name))
                try:
                    trans = buff.lookup_transform('base_link', joint.child, rospy.Time(), rospy.Duration(10))
                except tf2_ros.TransformException as e:
                    raise Exception(e)
                translation = rosmsg_to_numpy(trans.transform.translation)
                rot = rosmsg_to_numpy(trans.transform.rotation)
                yaw = euler_from_quaternion(rot)[2]
                names.append(transmission.name[0:find])
                positions.append(translation[0:2])
                angles.append(yaw)
                joints.append(joint.name)
                if limit != -1 and joint.limit.effort != limit:
                    raise Exception('Thruster {} had a different limit, cannot proceed'.format(joint.name))
                limit = joint.limit.effort

        return cls(names, positions, angles, ratio, limit, joints=joints)

    def thrusts_to_wrench(self, thrusts):
        '''
        Given a the 4 thrust commands in effort units, returns the
        equivilant
        param thrusts: np float array of thruster efforts in order of ThrusterMap.THRUSTERS
        returns: wrench (x, y, torque about z) force/torque applied to boat
        '''
        return self.effort_to_force(np.linalg.lstsq(self.thruster_matrix_inv, thrusts)[0])

    def wrench_to_thrusts(self, wrench):
        '''
        Given a wrench (force/torque applied to boat) returns a set of thruster
        efforts that will achieve this wrench or the closest to it possible while
        respecting effort limits.

        param wrench: np float array of force/torque (x, y, torque about z) in Newtons/Newton*Meters
        returns: wrench (x, y, torque) force/torque applied to boat
        '''
        return self.force_to_effort(np.linalg.lstsq(self.thruster_matrix, wrench)[0])

    def force_to_effort(self, force):
        '''
        Maps a list of thruster forces to their corposponding effort units
        '''
        return np.clip(force * self.effort_ratio, -self.effort_limit, self.effort_limit)

    def effort_to_force(self, effort):
        '''
        Maps a list of thrusts in effort units to the corosponding force in newtons
        '''
        return effort / self.effort_ratio
