#!/usr/bin/env python
from txros import util
from navigator_missions.navigator import Navigator
import numpy as np
from mil_tools import rosmsg_to_numpy
from twisted.internet import defer
import math
from mil_misc_tools import ThrowingArgumentParser
import tf2_ros


class ObstacleAvoid(Navigator):
    OBSTACLE_AVOID_POI = 'obstacle_course'
    DIFF_TOLERANCE = 10.0

    @classmethod
    def decode_parameters(cls, parameters):
        argv = parameters.split()
        return cls.parser.parse_args(argv)

    @classmethod
    def init(cls):
        parser = ThrowingArgumentParser(description='Start Gate Mission',
                                        usage='Default parameters: \'runtask ObstacleAvoid\'')
        parser.add_argument('-p', '--numpasses', type=int, default=3,
                            help='number of passes through the obstacle course to make')
        parser.add_argument('-d', '--outoffset', type=int, default=10,
                            help='distance from the course in meters to traverse to')

        cls.parser = parser

    @util.cancellableInlineCallbacks
    def run(self, args):
        # Parse Arguments
        num_passes = args.numpasses
        out_offset = args.outoffset
        
        yield self.find_area()


        midpoints_close = self.get_midpoints(self.square[0], self.square[1], num_passes)
        midpoints_far = self.get_midpoints(self.square[3], self.square[2], num_passes)

        traverse_points = []

        for i in range(0, num_passes):
            ppoints_close = yield self.get_perpendicular_points(self.square[0], self.square[1], midpoints_close[i], out_offset)
            ppoints_far = yield self.get_perpendicular_points(self.square[3], self.square[2], midpoints_far[i], out_offset)

            if i % 2 == 0:
                traverse_points.append(ppoints_close[0])
                traverse_points.append(ppoints_far[1])
            else:
                traverse_points.append(ppoints_far[1])
                traverse_points.append(ppoints_close[0])

        print('\ntraverse points:')
        print(traverse_points)

        for i in range(0, len(traverse_points)):
            if i + 1 != len(traverse_points):
                yield self.move.set_position(traverse_points[i]).look_at(traverse_points[i + 1]).go()
            else:
                yield self.move.set_position(traverse_points[i]).go()

        
        self.send_feedback('Done with obstacle avoid!')

    @util.cancellableInlineCallbacks
    def find_area(self):
        self.send_feedback('Waiting for ' + self.OBSTACLE_AVOID_POI)
        position = yield self.poi.get(self.OBSTACLE_AVOID_POI)
        #self.send_feedback('Moving to {} at {}'.format(poi, position[0:2]))
        
        self.square = None

        white_totems = yield self.get_sorted_objects("totem_white")
        white_totems = white_totems[1]
        for totem1 in white_totems:
            for totem2 in white_totems:
                if np.array_equal(totem1, totem2):
                    continue
                for totem3 in white_totems:
                    if np.array_equal(totem1, totem3) or np.array_equal(totem2, totem3):
                        continue
                    for totem4 in white_totems:
                        if np.array_equal(totem1, totem4) or np.array_equal(totem2, totem4) or np.array_equal(totem3, totem4):
                            continue

                        edge12 = np.linalg.norm(totem2 - totem1)
                        edge23 = np.linalg.norm(totem3 - totem2)
                        edge34 = np.linalg.norm(totem4 - totem3)
                        edge41 = np.linalg.norm(totem1 - totem4)
                        edge13 = np.linalg.norm(totem3 - totem1)
                        edge24 = np.linalg.norm(totem4 - totem2)
                        edges = np.array([edge12, edge23, edge34, edge41])
                        diagonals = np.array([edge13, edge24])

                        avg = np.average(edges)

                        diffs = np.absolute(edges - np.array([avg, avg, avg, avg]))
                        maxdif = diffs[np.argmax(diffs)]

                        if maxdif > self.DIFF_TOLERANCE:
                            print('failed with {}'.format(maxdif))
                            continue

                        if abs(diagonals[0] - diagonals[1]) > self.DIFF_TOLERANCE:
                            print('failed with {}'.format(maxdif))
                            continue

                        print('passed with {}'.format(maxdif))
                        self.square = (totem1, totem2, totem3, totem4)
                        break

                    if self.square != None:
                        break
                if self.square != None:
                    break
            if self.square != None:
                break

        if self.square == None:
            self.send_feedback('No square found! Cancelling')
            defer.returnValue(None)

        self.send_feedback('Found square, good to go')
        print(self.square)
        
        defer.returnValue(None)

    @staticmethod
    def get_midpoints(linep1, linep2, num):
        vec = linep2 - linep1
        vec /= (num + 1)

        points = []

        pt = linep1
        for i in range(0, num):
            pt = pt + vec
            points.append(pt)

        return points


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

    @util.cancellableInlineCallbacks
    def get_perpendicular_points(self, linep1, linep2, point, offset_distance):
        # Find the perpendicular line
        line = self.line(linep1, linep2)
        perpendicular_vector = self.perpendicular(line)

        boat_pose = yield self.tx_pose
        boat_pose = boat_pose[0]

        # Find the two points on either side of the line
        perpendicular_points = [(point[0] + perpendicular_vector[0] * offset_distance,
                                 point[1] + perpendicular_vector[1] * offset_distance),
                                (point[0] + perpendicular_vector[0] * -offset_distance,
                                 point[1] + perpendicular_vector[1] * -offset_distance)]

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