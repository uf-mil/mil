#!/usr/bin/env python3
from typing import Callable, List, Optional, Tuple

import numpy as np
import rospy
import tf2_ros
from mil_tools import rosmsg_to_numpy
from tf.transformations import euler_from_quaternion
from urdf_parser_py.urdf import URDF


def vrx_force_to_command_scalar(force):

    if force > 250:
        return 1.0
    elif force < -100:
        return -1.0

    # vrx: command->force | command > 0.01
    #   0.01+(59.82-0.01)/((0.56+exp(-5.0(x-0.28)))^(1/0.38))
    # vrx inverse: force->command | force > 3.27398
    #   -0.2 log(-0.246597 (0.56 - 4.73341/(-0.01 + x)^0.38))
    elif force > 3.27398:
        return -0.2 * np.log(-0.246597 * (0.56 - (4.73341 / ((-0.01 + force) ** 0.38))))
    # vrx: command->force | command < 0.01
    #   -199.13 + (-0.09+199.13)/((0.99 + exp(-8.84*(x+0.57)))^(1/5.34))
    # vrx inverse: force->command | force < 3.27398
    #   -0.113122 log(-154.285 (0.99 - (1.88948x10^12)/(199.13 + x)^5.34))
    elif force < 0:
        return -0.113122 * np.log(
            -154.285 * (0.99 - ((1.88948 * 10**12) / ((199.13 + force) ** 5.34)))
        )
    else:
        # approx broken range as strait line with 0.01cmd/3.2N
        return (0.01 / 3.27398) * force


vrx_force_to_command = np.vectorize(vrx_force_to_command_scalar)


def generate_linear_force_to_command(ratio):
    def force_to_command(force):
        return force * ratio

    return force_to_command


class ThrusterMap:
    """
    Helper class to map between global body forces / torques and thruster outputs, which are in a
    arbitrary effort unit. See thruster_mapper_node.py for usage example.
    """

    def __init__(
        self,
        names: List[str],
        positions: List[Tuple[float, float]],
        angles: List[float],
        force_to_command: Callable,
        force_limit: Tuple[float, float],
        com: np.ndarray = np.zeros(2),
        joints: Optional[List[str]] = None,
    ):
        """
        Creates a ThrusterMapper instance.

        Args:
            positions: List[Tuple[float, float]] - List of x, y positions for each
              thuster [(x, y), (x,y), ...], in order of ThurserMap.THRUSTERS, meters
            angles: list of angles for each thruster [theta, theta, ...]
              in order of ThurserMap.THRUSTERS, radians
            force_to_command: function to convert a vector of forces to the same size
              vector in command units (actuator specific)
            force_limit: (MAX_FORWARD, MAX_REVERSE) maximum force in either direction
              that should be commanded to a thruster, in newtons
            com: offset of boat's true center of mass to the frame thruster positions
              are given (base_link) defaults to (0, 0, 0) in case we don't know com

        The mapping from body wrench to individual thrusts or visa versa is a simple least square solver.
        Bibliography:
          [1] Christiaan De With
              "Optimal Thrust Allocation Methods for Dynamic Positioning of Ships"
              see: http://repository.tudelft.nl/assets/uuid:4c9685ac-3f76-41c0-bae5-a2a96f4d757e/DP_Report_FINAL.pdf

        TODO:
          - dynamic reconfigure ???
          - implement a more interesting solver, which attempts to minimize energy output like SubjuGator
        """
        self.names = names
        self.joints = joints
        self._force_to_command = force_to_command
        self.force_limit = force_limit
        if len(self.force_limit) != 2 or self.force_limit[1] > self.force_limit[0]:
            raise Exception(f"self.force_limit {self.force_limit} is invalid")

        """ Iterate through thruster positions and create thruster trans matrix"""
        thruster_matrix = []
        # loop through all sub positions and compute columns of A
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

        # returns a matrix made of the thruster columns
        self.thruster_matrix = np.hstack(thruster_matrix)
        self.thruster_matrix_inv = np.linalg.pinv(
            self.thruster_matrix
        )  # Magical numpy pseudoinverse

    @classmethod
    def from_vrx_urdf(cls, urdf_string: str):
        urdf = URDF.from_xml_string(urdf_string)
        buff = tf2_ros.Buffer()
        listener = tf2_ros.TransformListener(buff)  # noqa
        names = []
        positions = []
        angles = []
        for link in urdf.links:
            find = link.name.find("_propeller_link")
            if find == -1:
                continue
            name = link.name[:find]
            try:
                trans = buff.lookup_transform(
                    "wamv/base_link", link.name, rospy.Time(), rospy.Duration(10)
                )
            except tf2_ros.TransformException as e:
                raise Exception(e)
            translation = rosmsg_to_numpy(trans.transform.translation)
            rot = rosmsg_to_numpy(trans.transform.rotation)
            yaw = euler_from_quaternion(rot)[2]
            names.append(name)
            positions.append(translation[0:2])
            angles.append(yaw)
        return cls(names, positions, angles, vrx_force_to_command, (250.0, -100.0))

    @classmethod
    def from_urdf(
        cls, urdf_string: str, transmission_suffix: str = "_thruster_transmission"
    ):
        """
        Load from an URDF string. Expects each thruster to be connected a transmission ending in the specified suffix.
        A transform between the propeller joint and base_link must be available
        """
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
                    raise Exception(
                        "Transmission {} does not have 1 joint".format(
                            transmission.name
                        )
                    )
                if len(transmission.actuators) != 1:
                    raise Exception(
                        "Transmission {} does not have 1 actuator".format(
                            transmission.name
                        )
                    )
                t_ratio = transmission.actuators[0].mechanicalReduction
                if ratio != -1 and ratio != t_ratio:
                    raise Exception(
                        "Transmission {} has a different reduction ratio (not supported)".format(
                            t_ratio
                        )
                    )
                ratio = t_ratio
                joint = None
                for t_joint in urdf.joints:
                    if t_joint.name == transmission.joints[0].name:
                        joint = t_joint
                if joint is None:
                    rospy.logerr(
                        "Transmission joint {} not found".format(
                            transmission.joints[0].name
                        )
                    )
                try:
                    trans = buff.lookup_transform(
                        "wamv/base_link", joint.child, rospy.Time(), rospy.Duration(10)
                    )
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
                    raise Exception(
                        "Thruster {} had a different limit, cannot proceed".format(
                            joint.name
                        )
                    )
                limit = joint.limit.effort
        limit_tuple = (limit, -limit)
        return cls(
            names,
            positions,
            angles,
            generate_linear_force_to_command(ratio),
            limit_tuple,
            joints=joints,
        )

    def thrusts_to_wrench(self, thrusts):
        """
        Given a the 4 thrust commands in effort units, returns the
        equivalent
        param thrusts: np float array of thruster efforts in order of ThrusterMap.THRUSTERS
        returns: wrench (x, y, torque about z) force/torque applied to boat
        """
        raise Exception(
            "Unimplemented. Please file an issue if you encounter this error"
        )
        return self.effort_to_force(
            np.linalg.lstsq(self.thruster_matrix_inv, thrusts)[0]
        )

    def wrench_to_thrusts(self, wrench: np.ndarray):
        """
        Given a wrench (force/torque applied to boat) returns a set of thruster
        efforts that will achieve this wrench or the closest to it possible while
        respecting effort limits.

        param wrench: np float array of force/torque (x, y, torque about z) in Newtons/Newton*Meters
        returns: wrench (x, y, torque) force/torque applied to boat
        """
        return self.force_to_command(
            np.linalg.lstsq(self.thruster_matrix, wrench, rcond=-1)[0]
        )

    def force_to_command(self, force):
        """
        Maps a list of thruster forces to their corposponding effort units
        """
        return self._force_to_command(
            np.clip(force, self.force_limit[1], self.force_limit[0])
        )

    def effort_to_force(self, effort):
        """
        This command is currently unimplemented and will raise an immediate error.

        Maps a list of thrusts in effort units to the corresponding force in newtons.
        """
        raise Exception(
            "Unimplemented. Please file an issue if you encounter this error"
        )
        return effort / self.effort_ratio
