#!/usr/bin/env python
from __future__ import division

import txros
from twisted.internet import defer

from mil_misc_tools import text_effects

from .sub_singleton import SubjuGator, SonarObjects
from mil_ros_tools import rosmsg_to_numpy
from scipy.spatial import distance

import numpy as np

fprint = text_effects.FprintFactory(
    title="START_GATE", msg_color="cyan").fprint

SPEED = 0.6

CAREFUL_SPEED = 0.3

# How many meters to pass the gate by
DIST_AFTER_GATE = 1

RIGHT_OR_LEFT = 1


class StartGate(SubjuGator):

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        fprint('Waiting for odom')
        yield self.tx_pose()
        fprint('Found odom')
        fprint('Begin search for gates')
        rotate_start = self.move.zero_roll_and_pitch()
        for i in range(4):
            # Search 4 quadrants seperated by 90 degrees for the gate
            fprint('Searching {} degrees'.format(90 * i))
            yield rotate_start.yaw_right_deg(90 * i).go(speed=CAREFUL_SPEED)
            start = self.move.zero_roll_and_pitch()
            # Pitch up and down to populate pointcloud
            so = SonarObjects(self, [start.pitch_down_deg(7), start] * 5)
            transform = yield self._tf_listener.get_transform('/map', '/base_link')
            # [1, 0, 0] is front vector for self
            ray = transform._q_mat.dot(np.array([1, 0, 0]))
            # Start scan and search
            res = yield so.start_search_in_cone(
                transform._p,
                ray,
                angle_tol=60,
                distance_tol=11,
                speed=0.1,
                clear=True,
                c_func=self.find_gate)
            fprint('Found {} objects'.format(len(res.objects)))
            # Didn't find enough objects
            if len(res.objects) < 2:
                fprint('No objects')
                del so
                continue
            # Search for two objects that satisfy the gate
            gate_points = self.find_gate(res.objects, ray)
            if gate_points is None:
                fprint('No valid gates')
                del so
                continue
            # Break if all the checks passed for gate
            break
        if gate_points is None:
            fprint('Returning')
            yield rotate_start.go()
            defer.returnValue(False)

        # Gate = 120 inches wide = 3.048 meters
        # Gate Center = 60 inches
        # Pole offset from center = 12 inches
        # SubjuGator = 31 inches wide
        # 48 in - 32 in = 16 => 16/2 = 8 inch
        # 12 + 8 = 20 inch offset from middle to enter small portion

        distance_btwn_gate = distance.euclidean(gate_points[0], gate_points[1])
        fprint(
            'Distance between poles: {} m = {} inch'.format(
                distance_btwn_gate,
                distance_btwn_gate * 39.3701))

        # Find midpoint between the two poles/objects
        mid_point = gate_points[0] + gate_points[1]
        mid_point = mid_point / 2
        # Offset z so we don't hit the bar
        mid_point[2] = mid_point[2] - 0.75
        fprint('Midpoint: {}'.format(mid_point))

        fprint('Looking at gate', msg_color='yellow')
        yield self.move.look_at(mid_point).go(speed=CAREFUL_SPEED)

        normal = mid_point - self.pose.position
        normal[2] = 0
        normal = normal / np.linalg.norm(normal)
        fprint('Normal {}'.format(normal))

        if gate_points[0].dot(normal) < 0:
            right_gate = gate_points[0]
            left_gate = gate_points[1]
        else:
            right_gate = gate_points[1]
            left_gate = gate_points[0]

        offset_dir = right_gate - left_gate
        offset_dir = offset_dir / np.linalg.norm(offset_dir)

        offset_dir = offset_dir * 0.508 * RIGHT_OR_LEFT
        goal_point = mid_point + offset_dir

        fprint('Goalpoint: {}'.format(goal_point))

        fprint('Moving in front of goalpoint!', msg_color='yellow')
        yield self.move.set_position(goal_point - 2 * normal).look_at(goal_point).go(speed=SPEED)

        fprint('Style on dem haters!', msg_color='yellow')
        yield self.move.set_position(goal_point).yaw_right_deg(179).go(speed=CAREFUL_SPEED)

        fprint('Moving past the gate', msg_color='yellow')
        yield self.move.set_position(goal_point + DIST_AFTER_GATE * normal).yaw_right_deg(179).zero_roll_and_pitch().go(speed=CAREFUL_SPEED)
        defer.returnValue(True)

    def find_gate(self, objects,
                  ray,
                  min_distance_away=2.85,
                  max_distance_away=3.3,
                  perp_threshold=0.5,
                  depth_threshold=1):
        '''
        find_gate: search for two objects that satisfy critria

        Parameters:
        ray: direction we expect gate to be near
        min_distance_away: minimum distance for the two poles
        max_distance_away: max distance the two objects can be away from each other
        perp_threshold: max dot product value for perpindicular test with ray
        depth_threshold: make sure the two objects have close enough depth
        '''
        for o in objects:
            p = rosmsg_to_numpy(o.pose.position)
            for o2 in objects:
                if o2 is o:
                    continue
                p2 = rosmsg_to_numpy(o2.pose.position)
                if distance.euclidean(p, p2) > max_distance_away:
                    fprint('Poles too far away. Distance {}'.format(
                        distance.euclidean(p, p2)))
                    continue
                if distance.euclidean(p, p2) < min_distance_away:
                    fprint('Poles too close. Distance {}'.format(
                        distance.euclidean(p, p2)))
                    continue
                line = p - p2
                perp = line.dot(ray)
                perp = perp / np.linalg.norm(perp)
                if not (-perp_threshold <= perp <= perp_threshold):
                    fprint('Not perpendicular. Dot {}'.format(perp))
                    pass
                    # continue
                print('Dist {}'.format(line))
                if abs(line[2] > depth_threshold):
                    print('Not similar height. Height: {}. Thresh: '.format(
                        line[2], depth_threshold))
                    continue
                if abs(line[0]) < 1 and abs(line[1]) < 1:
                    fprint('Objects on top of one another. x {}, y {}'.format(
                        line[0], line[1]))
                    continue
                return (p, p2)
        return None
