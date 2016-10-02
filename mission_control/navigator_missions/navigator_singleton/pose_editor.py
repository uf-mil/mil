from __future__ import division
import warnings

import numpy as np
from tf import transformations
from nav_msgs.msg import Odometry
from uf_common.msg import PoseTwistStamped, PoseTwist, MoveToGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3, Twist
from navigator_tools import rosmsg_to_numpy, make_header, normalize
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef

import navigator_tools

UP = np.array([0.0, 0.0, 1.0], np.float64)
EAST, NORTH, WEST, SOUTH = [transformations.quaternion_about_axis(np.pi / 2 * i, UP) for i in xrange(4)]
UNITS = {'m': 1, 'ft': 0.3048, 'yard': 0.9144, 'rad': 1, 'deg': 0.0174533}

def normalized(x):
    x = np.array(x)
    if max(map(abs, x)) == 0:
        warnings.warn('Normalizing zero-length vector to random unit vector')
        x = np.random.standard_normal(x.shape)
    x = x / max(map(abs, x))
    x = x / np.linalg.norm(x)
    return x


def get_perpendicular(a, b=None):
    a = np.array(a)
    if max(map(abs, a)) == 0:
        if b is not None:
            return get_perpendicular(b)
        return normalized(np.random.standard_normal(a.shape))

    if b is None:
        b = np.random.standard_normal(a.shape)
    b = np.array(b)
    x = np.cross(a, b)
    if max(map(abs, x)) == 0:
        return get_perpendicular(a)
    return normalized(x)


def triad((a1, a2), (b1, b2)):
    # returns quaternion that rotates b1 to a1 and b2 near a2
    # can get orientation by passing in (global, local)
    aa = get_perpendicular(a1, a2)
    A = np.array([normalized(a1), aa, normalized(np.cross(a1, aa))])
    bb = get_perpendicular(b1, b2)
    B = np.array([normalized(b1), bb, normalized(np.cross(b1, bb))])
    rot = A.T.dot(B)
    return transformations.quaternion_from_matrix(
        [(a, b, c, 0) for a, b, c in rot] +
        [(0, 0, 0, 1)])


def look_at(forward, upish=UP):
    # assumes standard forward-left-up body coordinate system
    return triad((forward, upish), ([1, 0, 0], UP))


def look_at_without_pitching(forwardish, up=UP):
    # assumes standard forward-left-up body coordinate system
    return triad((up, forwardish), (UP, [1, 0, 0]))


def look_at_camera(forward, upish=UP):
    # assumes camera right-down-forward coordinate system
    return triad((forward, upish), (UP, [0, -1, 0]))

def get_valid_point(nav, point):
    if nav.enu_bounds is None:
        return point

    # Magic that finds if a point is in a polygon
    ab = np.subtract(nav.enu_bounds[0], nav.enu_bounds[1])[:2]
    ac = np.subtract(nav.enu_bounds[0], nav.enu_bounds[2])[:2]
    am = np.subtract(nav.enu_bounds[0], point)[:2]
    if np.dot(ab, ac) > .1:
        ac = np.subtract(nav.enu_bounds[0], nav.enu_bounds[3])[:2]

    if 0 <= np.dot(ab, am) <= np.dot(ab, ab) and 0 <= np.dot(am, ac) <= np.dot(ac, ac):
        # The point was okay - in bounds
        return point
    else:
        print "INVAILD TARGET POINT DETECTED"
        # TODO: Make boat go to edge
        return nav.pose[0]

class PoseEditor2(object):
    '''
    Used to chain movements together
    ex:
        yield p.forward(2, 'm').down(1, 'ft').yaw_left(50, 'deg').go()
        Will move forward 2 meters, down 1 foot, all while yawing left 50 degrees.

    ex:
        movement = p.yaw_right(.707)
        -- -- -- -- --
        yield movement.go(speed=1)
        Will yaw right .707 radians from the original orientation regardless of the current orientation
    '''
    def __init__(self, nav, pose):
        self.nav = nav

        # Right now position is stored as a 3d vector - should this be changed?
        self.position, self.orientation = pose

    def __repr__(self):
        return "p: {}, q: {}".format(self.position, self.orientation)

    @property
    def _rot(self):
        return transformations.quaternion_matrix(self.orientation)[:3, :3]

    @property
    def pose(self):
        return [self.position, self.orientation]

    @property
    def distance(self):
        diff = self.position - self.nav.pose[0]
        return np.linalg.norm(diff)

    def go(self, *args, **kwargs):
        # NOTE: C3 doesn't seems to handle different frames, so make sure all movements are in C3's
        #       fixed frame.
        self.position = get_valid_point(self.nav, self.position)
        self.nav._pose_pub.publish(PoseStamped(header=navigator_tools.make_header(frame='enu'),
                                               pose=navigator_tools.numpy_quat_pair_to_pose(*self.pose)))

        goal = self.nav._moveto_action_client.send_goal(self.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def set_position(self, position):
        return PoseEditor2(self.nav, [np.array(position), np.array(self.orientation)])

    def rel_position(self, rel_pos):
        position = self.position + self._rot.dot(np.array(rel_pos))
        return self.set_position(position)

    def forward(self, dist, unit='m'):
        return self.rel_position([dist * UNITS[unit], 0, 0])

    def backward(self, dist, unit='m'):
        return self.rel_position([-dist * UNITS[unit], 0, 0])

    def left(self, dist, unit='m'):
        return self.rel_position([0, dist * UNITS[unit], 0])

    def right(self, dist, unit='m'):
        return self.rel_position([0, -dist  * UNITS[unit], 0])

    # Orientation
    def set_orientation(self, orientation):
        if orientation.shape == (4, 4):
            # We're getting a homogeneous rotation matrix - not a quaternion
            orientation = transformations.quaternion_from_matrix(orientation)

        return PoseEditor2(self.nav, [self.position, orientation])

    def yaw_left(self, angle, unit='rad'):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle * UNITS[unit], UP),
            self.orientation
        ))

    def yaw_right(self, angle, unit='rad'):
        return self.yaw_left(-angle, unit)

    # ====== Some more advanced movements =========================

    def look_at_rel(self, rel_point):
        return self.set_orientation(look_at_without_pitching(rel_point))  # Using no pitch here since we are 2D

    def look_at(self, point):
        return self.look_at_rel(point - self.position)

    def to_lat_long(self, lat, lon, alt=0):
        '''
        Go to a lat long position and keep the same orientation
        Note: lat and long need to be degrees
        '''
        ecef_pos, enu_pos = self.nav.ecef_pose[0], self.nav.pose[0]

        # These functions want radians
        lat, lon = np.radians([lat, lon], dtype=np.float64)
        ecef_vector = ecef_from_latlongheight(lat, lon, alt) - ecef_pos
        enu_vector = enu_from_ecef(ecef_vector, ecef_pos)
        enu_vector[2] = 0  # We don't want to move in the z at all
        return self.set_position(enu_pos + enu_vector)

    def circle_point(self, point, radius, granularity=8, theta_offset=0):
        '''
        Circle a point whilst looking at it
        This returns a generator, so for use:

            circle = navigator.move.circle_point([1,2,0], 5)
            for p in circle:
                yield p.go()

        '''
        point = np.array(point)
        angle_incrment = 2 * np.pi / granularity
        sprinkles = transformations.euler_matrix(0, 0, angle_incrment)[:3, :3]

        # Find first point to go to using boat rotation
        next_point = np.append(normalize(self.nav.pose[0][:2] - point[:2]), 0)  # Doing this in 2d

        for i in range(granularity + 1):
            new = point + radius * next_point
            yield self.set_position(new).look_at(point).yaw_left(theta_offset)  # Remember this is a generator - not a twisted yield
            next_point = sprinkles.dot(next_point)

        yield self.set_position(new).look_at(point).yaw_left(theta_offset)

    # When C3 gets replaced, these may go away
    def as_MoveToGoal(self, linear=[0, 0, 0], angular=[0, 0, 0], **kwargs):
        return MoveToGoal(
            header=make_header(),
            posetwist=self.as_PoseTwist(linear, angular),
            **kwargs
        )

    def as_Pose(self):
        return Pose(
            position=Point(*np.append(self.position[:2], 0)),  # Don't set waypoints out of the water plane
            orientation=Quaternion(*self.orientation),
        )

    def as_PoseTwist(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        return PoseTwist(
            pose=self.as_Pose(),
            twist=Twist(
                linear=Vector3(*linear),
                angular=Vector3(*angular),
            ),
        )