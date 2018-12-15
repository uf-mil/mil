#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math
from mil_misc_tools import ThrowingArgumentParser
import tf2_ros
from navigator_msgs.srv import MessageExtranceExitGateRequest, MessageExtranceExitGate


class EntranceGate(Navigator):
    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        class Range(object):
            '''
                https://stackoverflow.com/a/12117089
            '''
            def __init__(self, start, end):
                self.start = start
                self.end = end

            def __eq__(self, other):
                return self.start <= other <= self.end

        parser = ThrowingArgumentParser(description='Start Gate Mission',
                                        usage='''Default parameters: \'runtask EntranceGate
                                         --scandist 10 --speed 0.75 --scantime 10 --traversaldist 7\'''')
        parser.add_argument('-r', '--drift', action='store_true',
                            help='''setting results in a scan by listening at two points (two points scan mode,
                            not setting results in a scan by traversing the gates (pass scan mode)''')
        parser.add_argument('-m', '--multilateration', action='store_true',
                            help='setting enables multilateration-based scanning, otherwise uses intersecting lines')
        parser.add_argument('-c', '--scandist', type=int, default=6,
                            help='distance from the gates in meters to scan from')
        parser.add_argument('-s', '--speed', type=float, default=0.5, choices=[Range(0.0, 1.0)],
                            help='speed to move when scanning in pass pass mode')
        parser.add_argument('-k', '--nokill', action='store_false',
                            help='set to not kill thrusters during scan in two points scan mode')
        parser.add_argument('-t', '--scantime', type=int, default=10,
                            help='number of seconds to scan at each point for in two points scan mode')
        parser.add_argument('-d', '--traversaldist', type=int, default=8,
                            help='distance from each side of the gates to navigate to when crossing')
        parser.add_argument('-e', '--exit', action='store_true',
                            help='set to configure that this is the exit pass')
        cls.parser = parser

        cls.net_service_call = cls.nh.get_service_client('/entrance_exit_gate_message', MessageExtranceExitGate)

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments
        pass_scan = not args.drift
        traversal_distance = args.traversaldist

        self.initial_boat_pose = yield self.tx_pose
        self.initial_boat_pose = self.initial_boat_pose[0]

        # Find the gates
        self.gate_results = yield self.find_gates()
        self.gate_centers = self.gate_results[0]
        self.gates_line = self.gate_results[1]
        self.gate_totems = self.gate_results[2]

        # If nothing is found, just go through gate 2
        self.pinger_gate = 1

        if pass_scan:
            yield self.run_pass_scan(args)
        else:
            yield self.run_dual_scan(args)

        self.send_feedback('Gate identified as ' + str(self.pinger_gate + 1))
        if args.exit:
            msg = MessageExtranceExitGateRequest()
            if self.net_entrance_results is not None:
                msg.entrance_gate = self.net_entrance_results
            else:
                msg.entrance_gate = 2
            msg.exit_gate = self.pinger_gate + 1
            if self.net_stc_results is not None:
                msg.light_buoy_active = True
                msg.light_pattern = self.net_stc_results
            else:
                msg.light_buoy_active = False
                msg.light_pattern = 'RRR'
            self.net_service_call(msg)
        else:
            self.net_entrance_results = self.pinger_gate + 1

        # Calculate traversal points
        traversal_points = yield self.get_perpendicular_points(self.gate_centers[self.pinger_gate],
                                                               traversal_distance,
                                                               boat_pose=self.initial_boat_pose)

        # Go through the gate
        self.send_feedback('Navigating through gate')
        yield self.move.set_position(traversal_points[0]).look_at(traversal_points[1]).go()
        yield self.move.set_position(traversal_points[1]).go()
        self.send_feedback('Done with start gate!')

    @util.cancellableInlineCallbacks
    def run_dual_scan(self, args):
        # Parse Parameters
        uses_multilateration = args.multilateration
        scan_dist = args.scandist
        should_kill = args.nokill
        listen_time = args.scantime

        # Calculate scan points
        scan_points_0 = yield self.get_perpendicular_points(self.gate_totems[1], scan_dist)
        scan_points_1 = yield self.get_perpendicular_points(self.gate_totems[2], scan_dist)
        scan_point_0 = scan_points_0[0]
        scan_lookat_0 = scan_points_0[1]
        scan_point_1 = scan_points_1[0]
        scan_lookat_1 = scan_points_1[1]

        if uses_multilateration:
            # Reset Scan
            yield self.hydrophones.disable()
            yield self.hydrophones.reset()
        else:
            # Reset pings
            self.intersect_vectors = []

        # Execute Scan 1

        self.send_feedback('Navigating to scan start point 1')
        yield self.move.set_position(scan_point_0).look_at(scan_lookat_0).go()

        # If kill is enabled, kill the thrusters
        if should_kill:
            self.disable_thrusters()

        if uses_multilateration:
            # Turn on multilateration
            self.hydrophones.enable()
        else:
            # Enable ping counting
            self.hydrophones.set_callback(self.ping_recv)

        # Sleep
        yield self.nh.sleep(listen_time)

        if uses_multilateration:
            # Disable multilateration
            self.hydrophones.disable()
        else:
            # Disable ping counting
            self.hydrophones.set_callback(None)

        # If kill is enabled, revive the thrusters
        if should_kill:
            self.enable_autonomous()

        # Execute Scan 2

        self.send_feedback('Navigating to scan start point 2')
        yield self.move.set_position(scan_point_1).look_at(scan_lookat_1).go()

        # If kill is enabled, kill the thrusters
        if should_kill:
            self.disable_thrusters()

        if uses_multilateration:
            # Turn on multilateration
            self.hydrophones.enable()
        else:
            # Enable ping counting
            self.hydrophones.set_callback(self.ping_recv)

        # Sleep
        yield self.nh.sleep(listen_time)

        if uses_multilateration:
            # Disable multilateration
            self.hydrophones.disable()
        else:
            # Disable ping counting
            self.hydrophones.set_callback(None)

        # If kill is enabled, revive the thrusters
        if should_kill:
            self.enable_autonomous()

        # Calculate results
        if uses_multilateration:
            multilateration_results = yield self.get_multilateration_closest()
            self.pinger_gate = multilateration_results[0]
            multilateration_confidence = multilateration_results[1]

            self.send_feedback('Multilaterating Method has confidence of ' + str(multilateration_confidence))
        else:
            intersect_results = self.calculate_intersect()
            self.pinger_gate = intersect_results[0]
            intersect_confidence = intersect_results[1]

            self.send_feedback('Intersecting Method has confidence of ' + str(intersect_confidence))

    @util.cancellableInlineCallbacks
    def run_pass_scan(self, args):
        # Parse Parameters
        uses_multilateration = args.multilateration
        scan_dist = args.scandist
        speed = args.speed

        # Calculate scan points
        scan_points_0 = yield self.get_perpendicular_points(self.gate_centers[0], scan_dist)
        scan_points_1 = yield self.get_perpendicular_points(self.gate_centers[2], scan_dist)
        scan_points_2 = yield self.get_perpendicular_points(self.gate_totems[3], scan_dist)
        scan_point_0 = scan_points_0[0]
        scan_lookat_0 = scan_points_1[0]
        scan_point_1 = scan_points_1[0]
        scan_lookat_1 = scan_points_2[0]

        if uses_multilateration:
            # Reset Scan
            yield self.hydrophones.disable()
            yield self.hydrophones.reset()
        else:
            # Reset pings
            self.intersect_vectors = []

        # Execute Scan

        self.send_feedback('Navigating to scan start point')
        yield self.move.set_position(scan_point_0).look_at(scan_lookat_0).go()

        if uses_multilateration:
            # Turn on multilateration
            self.hydrophones.enable()
        else:
            # Enable ping counting
            self.hydrophones.set_callback(self.ping_recv)

        self.send_feedback('Navigating to scan end point')
        yield self.move.set_position(scan_point_1).look_at(scan_lookat_1).go(speed_factor=speed)

        if uses_multilateration:
            # Disable multilateration
            self.hydrophones.disable()
        else:
            # Disable ping counting
            self.hydrophones.set_callback(None)

        # Calculate results
        if uses_multilateration:
            multilateration_results = yield self.get_multilateration_closest()
            self.pinger_gate = multilateration_results[0]
            multilateration_confidence = multilateration_results[1]

            self.send_feedback('Multilaterating Method has confidence of ' + str(multilateration_confidence))
        else:
            intersect_results = self.calculate_intersect()
            self.pinger_gate = intersect_results[0]
            intersect_confidence = intersect_results[1]

            self.send_feedback('Intersecting Method has confidence of ' + str(intersect_confidence))

    '''

        Intersecting

    '''

    def calculate_intersect(self):
        gate_vector = np.array([self.gate_totems[0], self.gate_totems[3]])
        bucket = [0, 0, 0]
        gateDist = [0, 0, 0]
        for pinger_vector in self.intersect_vectors:
            # Calculate intersection between the gate and the pinger vector
            t, s = np.linalg.solve(np.array([gate_vector[1] - gate_vector[0], pinger_vector[0] - pinger_vector[1]]).T,
                                   pinger_vector[0] - gate_vector[0])
            intersection = (1 - t) * gate_vector[0] + t * gate_vector[1]

            # Calculate the distances from each gate center to the intersection
            gateDist[0] = np.linalg.norm(intersection - np.array(self.gate_centers[0]))
            gateDist[1] = np.linalg.norm(intersection - np.array(self.gate_centers[1]))
            gateDist[2] = np.linalg.norm(intersection - np.array(self.gate_centers[2]))

            # If the intersection is more than 15 meters from the center of the gates it is outside the gates
            if gateDist[1] > 16:
                # Ignore it
                continue

            # Calculate vectors to the edges of the gates and the intersect
            vec_corner1 = gate_vector[0] - pinger_vector[0]
            vec_corner1 = vec_corner1 / np.linalg.norm(vec_corner1)
            vec_corner4 = gate_vector[1] - pinger_vector[0]
            vec_corner4 = vec_corner4 / np.linalg.norm(vec_corner4)
            vec_intersect = intersection - pinger_vector[0]
            vec_intersect = vec_intersect / np.linalg.norm(vec_intersect)

            # Calculate the angle between the edges of the gates and the angle between an edge and the intersection
            ang_c1_c4 = np.arccos(np.clip(np.dot(vec_corner1, vec_corner4), -1.0, 1.0))
            ang_c1_i4 = np.arccos(np.clip(np.dot(vec_corner1, vec_corner4), -1.0, 1.0))

            # If the angles are out of range, throw out this intersection
            if ang_c1_c4 >= 0 and (ang_c1_i4 < 0 or ang_c1_i4 > ang_c1_c4):
                continue
            if ang_c1_c4 < 0 and (ang_c1_i4 > 0 or ang_c1_i4 < ang_c1_c4):
                continue

            # Determine which gate the intersection is in
            bucket_index = np.argmin(np.array(gateDist))
            bucket[bucket_index] = bucket[bucket_index] + 1

        # Determine which gate has the most hits
        pinger_gate = np.argmax(np.array(bucket))

        # Calculate the confidence
        confidence = bucket[pinger_gate] / (bucket[0] + bucket[1] + bucket[2])
        return (pinger_gate, confidence)

    @util.cancellableInlineCallbacks
    def ping_recv(self, p_message):
        try:
            # Transform the ping into enu
            hydrophones_to_enu = yield self.tf_listener.get_transform('enu', p_message.header.frame_id)
            hydrophones_origin = hydrophones_to_enu._p[0:2]
            heading = rosmsg_to_numpy(p_message.vector)
            heading_enu = hydrophones_to_enu.transform_vector(heading)
            heading_enu = heading_enu[0:2] / np.linalg.norm(heading_enu[0:2])

            # Track the ping
            self.intersect_vectors.append((hydrophones_origin[0:2], hydrophones_origin[0:2] + heading_enu[0:2]))
        except tf2_ros.TransformException, e:
            self.send_feedback('TF Exception: {}'.format(e))

    '''

        Multilaterating

    '''

    @util.cancellableInlineCallbacks
    def get_multilateration_closest(self):
        # Get the most recent multilateration position
        rospos = yield self.hydrophones.get_last_position()
        pinger_pos = rosmsg_to_numpy(rospos.point)[0:2]

        # Calculate the distances to each gate center
        distances = []
        for gate_center in self.gate_centers:
            distances.append((gate_center[0] - pinger_pos[0]) ** 2 + (gate_center[1] - pinger_pos[1]) ** 2)

        # Determine which gate is closest
        pinger_gate = np.argmin(np.array(distances))

        # Calculate the confidence
        confidence = (5 - distances[pinger_gate]) / 5.0
        defer.returnValue((pinger_gate, confidence))

    '''

        Math Utilities

    '''

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
    def perpendicular(L):
        '''
        Calculate the line perpendicular to another line
        '''
        v1 = [L[1], -L[0], 0]
        r = v1[0] ** 2 + v1[1] ** 2
        v1 = [v1[0] / r, v1[1] / r]
        v2 = [0, 0, 1]
        pvec = np.cross(v1, v2)
        r = math.sqrt(pvec[0] ** 2 + pvec[1] ** 2)
        return [pvec[0] / r, pvec[1] / r, 0]

    '''

        Perception Utilities

    '''

    @util.cancellableInlineCallbacks
    def find_gates(self):
        # Find each of the needed totems
        t1 = yield self.get_sorted_objects("totem_red", n=1)
        t1 = t1[1][0][:2]
        white_totems = yield self.get_sorted_objects("totem_white", n=2)
        t2 = white_totems[1][0][:2]
        t3 = white_totems[1][1][:2]
        t4 = yield self.get_sorted_objects("totem_green", n=1)
        t4 = t4[1][0][:2]

        # Make sure the two white totems get ordered properly
        if (t2[0] - t1[0]) ** 2 + (t2[1] - t1[1]) ** 2 < (t3[0] - t1[0]) ** 2 + (t3[1] - t1[1]) ** 2:
            gate_totems = [t1, t2, t3, t4]
        else:
            gate_totems = [t1, t3, t2, t4]

        # Calculate the center points of each gate
        gate_centers = [((gate_totems[0][0] + gate_totems[1][0]) / 2, (gate_totems[0][1] + gate_totems[1][1]) / 2),
                        ((gate_totems[1][0] + gate_totems[2][0]) / 2, (gate_totems[1][1] + gate_totems[2][1]) / 2),
                        ((gate_totems[2][0] + gate_totems[3][0]) / 2, (gate_totems[2][1] + gate_totems[3][1]) / 2)]

        # Calculate the line that goes through the gates
        gates_line = self.line(gate_centers[0], gate_centers[2])
        defer.returnValue((gate_centers, gates_line, gate_totems))

    '''

        Navigation Utilities

    '''

    @util.cancellableInlineCallbacks
    def get_perpendicular_points(self, center_point, offset_distance, boat_pose=None):
        # Find the perpendicular line
        perpendicular_vector = self.perpendicular(self.gates_line)

        if boat_pose is None:
            boat_pose = yield self.tx_pose
            boat_pose = boat_pose[0]

        # Find the two points on either side of the line
        perpendicular_points = [(center_point[0] + perpendicular_vector[0] * offset_distance,
                                 center_point[1] + perpendicular_vector[1] * offset_distance),
                                (center_point[0] + perpendicular_vector[0] * -offset_distance,
                                 center_point[1] + perpendicular_vector[1] * -offset_distance)]

        # Sort them such that the point on the same side of the boat is first
        if (perpendicular_points[0][0] - boat_pose[0]) ** 2 + (perpendicular_points[0][1] - boat_pose[1]) ** 2 > \
                (perpendicular_points[1][0] - boat_pose[0]) ** 2 + (perpendicular_points[1][1] - boat_pose[1]) ** 2:
            perpendicular_points = [perpendicular_points[1], perpendicular_points[0]]

        perpendicular_points_np = []

        # Turn the points into 3D numpy points with U=0
        for goal_point in perpendicular_points:
            point = np.array(goal_point)
            point = np.append(point, [0])
            perpendicular_points_np.append(point)

        defer.returnValue(perpendicular_points_np)
