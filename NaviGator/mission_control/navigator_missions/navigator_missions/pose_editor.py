from __future__ import division
import warnings

import numpy as np
from tf import transformations
from mil_msgs.msg import MoveToGoal, PoseTwist
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3, Twist
from mil_tools import make_header, normalize
from rawgps_common.gps import ecef_from_latlongheight, enu_from_ecef
from navigator_path_planner.msg import MoveGoal

import mil_tools
from mil_misc_tools.text_effects import fprint

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
    """
    Used to chain movements together

    ex:
        >>> res = yield p.forward(2, 'm').down(1, 'ft').yaw_left(50, 'deg').go()

        Will move forward 2 meters, down 1 foot, all while yawing left 50 degrees.

    ex:
        >>> movement = p.yaw_right(.707)
        >>> ...
        >>> res = yield movement.go(speed=1)

        Will yaw right .707 radians from the original orientation regardless of the current orientation

    Some special cases (these can't be chained):
        >>> circle = p.circle_point([0, 1, 0])
        >>> res = yield circle.go()

        Will circle the enu point [0, 1, 0] counter clockwise holding the current orientation and distance
        from the point.

        >>> res = yield p.spiral_point([10, 0], 'cw', meters_per_rev=2).go()

        Will spiral the enu point [10, 0, 0] in a clockwise direction increase it's current
        radius by `meters_per_rev` meters per revolution.
    """

    def __init__(self, nav, pose, **kwargs):
        self.nav = nav

        # Position and kwargs ultimatly passed into the final function
        self.position, self.orientation = pose
        self.kwargs = kwargs

        # Move result (should be a defered)
        self.result = None

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
        return np.linalg.norm(self.position - self.nav.pose[0])

    def go(self, *args, **kwargs):
        if self.nav.killed is True or self.nav.odom_loss is True:
            # What do we want to do with missions when the boat is killed
            fprint("Boat is killed, ignoring go command!", title="POSE_EDITOR", msg_color="red")

            class Res():
                failure_reason = 'boat_killed'

            return Res()

        if len(self.kwargs) > 0:
            kwargs = dict(kwargs.items() + self.kwargs.items())

        goal = self.nav._moveto_client.send_goal(self.as_MoveGoal(*args, **kwargs))
        self.result = goal.get_result()
        return self.result

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
        return self.rel_position([0, -dist * UNITS[unit], 0])

    def stop(self):
        return self.forward(0)

    # Orientation
    def set_orientation(self, orientation):
        orientation = np.array(orientation)
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

    def to_pose(self, pose):
        ''' Takes a Pose or PoseStamped '''
        if isinstance(pose, PoseStamped):
            pose = pose.pose

        position, orientation = mil_tools.pose_to_numpy(pose)
        return PoseEditor2(self.nav, [position, orientation])

    def to_lat_long(self, lat, lon, alt=0):
        """
        Goes to a lat long position and keep the same orientation
        Note: lat and long need to be degrees
        """
        ecef_pos, enu_pos = self.nav.ecef_pose[0], self.nav.pose[0]

        # These functions want radians
        lat, lon = np.radians([lat, lon], dtype=np.float64)
        ecef_vector = ecef_from_latlongheight(lat, lon, alt) - ecef_pos
        enu_vector = enu_from_ecef(ecef_vector, ecef_pos)
        enu_vector[2] = 0  # We don't want to move in the z at all
        return self.set_position(enu_pos + enu_vector)

    def spiral_point(self, point, direction='ccw', revolutions=1, meters_per_rev=0):
        """
        Sprials an ENU point.

        `point` is a 2d or 3d numpy point or geometery_msgs/PointStamped to spiral
        `direction` can be 'cw' or 'ccw' for clockwise or counter clockwise
        `meters_per_rev` is the number of meters per revolution to increase the radius of the spiral
        `revolutions` is the number of revolutions to complete

        NOTE: Don't use this function with other pose editor functions like you traditionally would
        """
        position = np.array([0, 0, meters_per_rev])
        sign_direction = 1 if direction == 'ccw' else -1  # Follows the right hand rule
        if hasattr(point, 'point'):
            focus = [point.point.x, point.point.y]
        else:
            focus = [point[0], point[1]]

        focus.append(sign_direction * revolutions)

        return PoseEditor2(self.nav, [position, [0, 0, 0, 1]], focus=np.array(focus), move_type=MoveGoal.SPIRAL)

    def circle_point(self, point, *args, **kwargs):
        return self.spiral_point(point, *args, **kwargs)

    def d_spiral_point(self, point, radius, granularity=8, revolutions=1, direction='ccw',
                       theta_offset=0, meters_per_rev=0):
        """
        Sprials a point using discrete moves
        This produces a generator
        """
        point = np.array(point)
        if direction == 'ccw':
            angle_incrment = 2 * np.pi / granularity
        else:
            angle_incrment = -2 * np.pi / granularity
        sprinkles = transformations.euler_matrix(0, 0, angle_incrment)[:3, :3]

        # Find first point to go to using boat rotation
        next_point = np.append(normalize(self.position[:2] - point[:2]), 0)  # Doing this in 2d
        radius_increment = meters_per_rev / granularity
        for i in range(granularity * revolutions + 1):
            new = point + radius * next_point
            radius += radius_increment

            yield self.set_position(new).look_at(point).yaw_left(theta_offset)
            next_point = sprinkles.dot(next_point)

        yield self.set_position(new).look_at(point).yaw_left(theta_offset)

    def d_circle_point(self, *args, **kwargs):
        """
        Circles a point whilst looking at it using discrete steps
        This produces a generator, so for use:

            circle = navigator.move.circle_point([1,2,0], 5)
            for p in circle:
                yield p.go()
        """
        return self.d_spiral_point(*args, **kwargs)

    def as_MoveToGoal(self, linear=[0, 0, 0], angular=[0, 0, 0], **kwargs):
        return MoveToGoal(
            header=make_header(),
            posetwist=self.as_PoseTwist(linear, angular),
            **kwargs
        )

    def as_MoveGoal(self, move_type=MoveGoal.DRIVE, **kwargs):
        if 'focus' in kwargs:
            if not isinstance(kwargs['focus'], Point):
                kwargs['focus'] = mil_tools.numpy_to_point(kwargs['focus'])

        if 'speed_factor' in kwargs and isinstance(kwargs['speed_factor'], float):
            # User wants a uniform speed factor
            sf = kwargs['speed_factor']
            kwargs['speed_factor'] = [sf, sf, sf]

        for key in kwargs.keys():
            if not hasattr(MoveGoal, key):
                fprint("MoveGoal msg doesn't have a field called '{}' you tried to set via kwargs.".format(key),
                       title="POSE_EDITOR", msg_color="red")
                del kwargs[key]

        return MoveGoal(
            goal=self.as_Pose(),
            move_type=move_type,
            **kwargs
        )

    def as_Pose(self):
        return Pose(
            position=Point(*np.array(self.position)),  # Don't set waypoints out of the water plane
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
