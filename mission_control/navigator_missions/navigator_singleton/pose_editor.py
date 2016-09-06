from __future__ import division
import warnings

import numpy as np
from tf import transformations
from nav_msgs.msg import Odometry
from uf_common.msg import PoseTwistStamped, PoseTwist, MoveToGoal
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3, Twist
from sub8_ros_tools import rosmsg_to_numpy, make_header
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef

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
    def __init__(self, nav, frame_id):
        self.nav = nav
        self.frame_id = frame_id

        self.position, self.orientation = nav.pose

    def __repr__(self):
        return "{} - p: {}, q: {}".format(self.frame_id, self.position, self.orientation)

    @property
    def _rot(self):
        return transformations.quaternion_matrix(self.orientation)[:3, :3]

    def go(self, *args, **kwargs):
        # NOTE: C3 doesn't seems to handle different frames, so make sure all movements are in C3's
        #       fixed frame.
        goal = self.nav._moveto_action_client.send_goal(self.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def set_position(self, position, unit='m'):
        self.position = np.array(position) * UNITS[unit]
        return self

    def rel_position(self, rel_pos, unit='m'):
        position = self.position + self._rot.dot(np.array(rel_pos))
        self.set_position(position, unit)
        return self

    def forward(self, dist, unit='m'):
        return self.rel_position([dist, 0, 0], unit)

    def backward(self, dist, unit='m'):
        return self.rel_position([-dist, 0, 0], unit)

    def left(self, dist, unit='m'):
        return self.rel_position([0, dist, 0], unit)

    def right(self, dist, unit='m'):
        return self.rel_position([0, -dist, 0], unit)

    def up(self, dist, unit='m'):
        return self.rel_position([0, 0, dist], unit)

    def down(self, dist, unit='m'):
        return self.rel_position([0, 0, -dist], unit)

    # Orientation
    def set_orientation(self, orientation):
        if orientation.shape == (4, 4):
            # We're getting a homogeneous rotation matrix - not a quaternion
            orientation = transformations.quaternion_from_matrix(orientation)
        self.orientation = orientation
        return self

    def yaw_left(self, angle, unit='rad'):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle * UNITS[unit], UP),
            self.orientation
        ))

    def yaw_right(self, angle, unit='rad'):
        return self.yaw_left(-angle, unit)

    def depth(self, depth, unit='m'):
        return self.set_position([self.position[0], self.position[1], -depth], unit)

    def roll_right(self, angle, unit='rad'):
        return self.set_orientation(transformations.quaternion_multiply(
            self.orientation,
            transformations.quaternion_about_axis(angle * UNITS[unit], [1, 0, 0]),
        ))

    def roll_left(self, angle, unit='rad'):
        return self.roll_right(-angle, unit)

    def zero_roll(self):
        return self.set_orientation(look_at(self.forward_vector))

    def pitch_down(self, angle, unit='rad'):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle * UNITS[unit], [0, 1, 0]),
            self.orientation,
        ))

    def pitch_up(self, angle, unit='rad'):
        return self.pitch_down(-angle, unit)

    def zero_roll_and_pitch(self):
        return self.set_orientation(look_at_without_pitching(self.forward_vector))

    def latlon(self, lat, lon, alt=0):
        '''
        Go to a lat lon position and keep the same orientation
        '''
        # These fuctions want radians
        lat, lon = np.radians([lat, lon])
        # Vector in ECEF frame
        v = ecef_from_latlongheight(lat, lon, alt) - self.nav.ecef_pose[0]
        # Convert the vector to ENU
        enu_vector = enu_from_ecef(v, self.nav.ecef_pose[0])
        enu_vector[2] = 0  # We don't want to move in the z at all.

        return self.rel_position(enu_vector)

    # When C3 gets replaced, these may go away
    def as_MoveToGoal(self, linear=[0, 0, 0], angular=[0, 0, 0], **kwargs):
        return MoveToGoal(
            header=make_header(self.frame_id),
            posetwist=self.as_PoseTwist(linear, angular),
            **kwargs
        )

    def as_Pose(self):
        return Pose(
            position=Point(*self.position),
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