#!/usr/bin/env python
from txros import util
from twisted.internet import defer
from navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from geometry_msgs.msg import Vector3Stamped


class PingerAndy(Navigator):
    '''
    Mission to run sonar start gate challenge using Andy's sonar system, which produces a vector pointing towards the
    '''
    @classmethod
    def init(cls):
        cls.pinger_heading = cls.nh.subscribe("/hydrophones/ping_direction", Vector3Stamped)

    @staticmethod
    def line(p1, p2):
        '''
        Return equation of a line given two 2D points
        https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
        '''
        A = (p1[1] - p2[1])
        B = (p2[0] - p1[0])
        C = (p1[0] * p2[1] - p2[0] * p1[1])
        return A, B, -C

    @staticmethod
    def intersection(L1, L2):
        '''
        Return point intersection (if it exsists) of two lines given their equations obtained from the line method
        https://stackoverflow.com/questions/20677795/how-do-i-compute-the-intersection-point-of-two-lines-in-python
        '''
        D = L1[0] * L2[1] - L1[1] * L2[0]
        Dx = L1[2] * L2[1] - L1[1] * L2[2]
        Dy = L1[0] * L2[2] - L1[2] * L2[0]
        if D != 0:
            x = Dx / D
            y = Dy / D
            return x, y
        else:
            return None

    @util.cancellableInlineCallbacks
    def get_gates(self):
        totems = []
        for i in range(4):
            while True:
                self.send_feedback('Click on totem {} in rviz'.format(i + 1))
                point = yield self.rviz_point.get_next_message()
                if point.header.frame_id != 'enu':
                    self.send_feedback('Point is not in ENU.\
                         Please switch rviz frame to ENU or tell kevin to support other frames.')
                    continue
                break
            self.send_feedback('Recieved point for totem {}'.format(i + 1))
            point = rosmsg_to_numpy(point.point)
            point[2] = 0.0
            totems.append(np.array(point))
        # Create list of gates halfway between each pair of totems
        gates = []
        for i in range(3):
            gates.append((totems[i] + totems[i + 1]) / 2.0)
        defer.returnValue(gates)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Get position of 3 gates based on position of totems
        gates = yield self.get_gates()

        # Get heading towards pinger from Andy hydrophone system
        self.send_feedback('All gates clicked on! Waiting for pinger heading...')
        heading = yield self.pinger_heading.get_next_message()
        self.send_feedback('Recieved pinger heading')

        # Convert heading and hydophones from to enu
        hydrophones_to_enu = yield self.tf_listener.get_transform('enu', heading.header.frame_id)
        hydrophones_origin = hydrophones_to_enu._p[0:2]
        heading = rosmsg_to_numpy(heading.vector)
        heading_enu = hydrophones_to_enu.transform_vector(heading)
        heading_enu = heading_enu[0:2] / np.linalg.norm(heading_enu[0:2])

        pinger_line = self.line(hydrophones_origin, hydrophones_origin + heading_enu)
        gates_line = self.line(gates[0], gates[-1])

        # Find intersection of these two lines. This is the approximate position of the pinger
        intersection = self.intersection(pinger_line, gates_line)
        if intersection is None:
            raise Exception('No intersection')
        self.send_feedback('Pinger is roughly at {}'.format(intersection))

        distances = []
        for gate in gates:
            distances.append(np.linalg.norm(gate[0:2] - intersection))
        argmin = np.argmin(np.array(distances))
        self.send_feedback('Pinger is likely at gate {}'.format(argmin + 1))

        gate = gates[argmin][:2]

        between_vector = (gates[0] - gates[-1])[:2]
        # Rotate that vector to point through the buoys
        c = np.cos(np.radians(90))
        s = np.sin(np.radians(90))
        R = np.array([[c, -s], [s, c]])
        direction_vector = R.dot(between_vector)
        direction_vector /= np.linalg.norm(direction_vector)
        position = self.pose[0][:2]
        if np.linalg.norm(position - (gate + direction_vector)) > np.linalg.norm(position - (gate - direction_vector)):
            direction_vector = -direction_vector

        before_distance = 3.0
        after_distance = 5.0
        before = np.append(gate + direction_vector * before_distance, 0)
        after = np.append(gate - direction_vector * after_distance, 0)

        self.send_feedback('Moving in front of gate')
        yield self.move.set_position(before).look_at(after).go()
        self.send_feedback('Going through')
        yield self.move.set_position(after).go()

        defer.returnValue('My god it actually worked!')
