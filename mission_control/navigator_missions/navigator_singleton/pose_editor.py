from __future__ import division
import warnings

import numpy as np
#import rospy
from tf import transformations
from nav_msgs.msg import Odometry
from uf_common.msg import PoseTwistStamped, PoseTwist, MoveToGoal
from std_msgs.msg import Header
import geometry_msgs
from geometry_msgs.msg import Pose, PoseStamped, Quaternion, Point, Vector3, Twist
from sub8_ros_tools import rosmsg_to_numpy, make_header


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


def quat_to_rotvec(q):
    if q[3] < 0:
        q = -q
    q = transformations.unit_vector(q)
    angle = np.arccos(q[3]) * 2
    axis = normalized(q[0:3])
    return axis * angle


def rotvec_to_quat(rotvec):
    return transformations.quaternion_about_axis(np.linalg.norm(rotvec), rotvec)


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


def test_triad():
    q = transformations.random_quaternion()

    a = np.random.standard_normal(3)
    b = np.random.standard_normal(3)

    m = transformations.quaternion_matrix(q)[:3, :3]
    q_ = triad((m.dot(a), m.dot(b)), (a, b))

    assert np.linalg.norm(quat_to_rotvec(
        transformations.quaternion_multiply(
            q,
            transformations.quaternion_inverse(q_),
        )
    )) < 1e-6


def look_at(forward, upish=UP):
    # assumes standard forward-left-up body coordinate system
    return triad((forward, upish), ([1, 0, 0], UP))


def look_at_without_pitching(forwardish, up=UP):
    # assumes standard forward-left-up body coordinate system
    return triad((up, forwardish), (UP, [1, 0, 0]))


def look_at_camera(forward, upish=UP):
    # assumes camera right-down-forward coordinate system
    return triad((forward, upish), (UP, [0, -1, 0]))


def safe_wait_for_message(topic, topic_type):
    while True:
        try:
            return rospy.wait_for_message(topic, topic_type, .5)
        except rospy.exceptions.ROSException, e:
            if 'timeout' not in e.message:
                raise
            print topic, 'wait_for_message timed out!'


class PoseEditor2(object):
    '''
    All pose editor commands are treated realative to the position and rotation at the time go() is called UNLESS set_start()
        is called beforehand, then movements will be calculated relative to that point.

    ex:
        p.forward(2, 'm').down(1, 'ft').yaw_left(50, 'deg').go()
        Will move forward 2 meters, down 1 foot, all while yawing left 50 degrees.

    ex:
        movement = p.set_start().yaw_right(.707)
        -- - -- - -- - --
        movement.go(speed=1)
        Will yaw right .707 radians from the original orientation regardless of the current orientation
    '''
    def __init__(self, ac, frame_id, position, orientation, tf=None):        
        self.action_client = ac
        self.frame_id = frame_id

        self.position = np.array(position)
        self.orientation = np.array(orientation)

        self._transformer = tf

    def __repr__(self):
        return "{} - p: {}, q: {}".format(self.frame_id, self.position, self.orientation)

    @property
    def _rot(self):
        return transformations.quaternion_matrix(self.orientation)[:3, :3]

    def go(self, *args, **kwargs):
        goal = self.action_client.send_goal(self.as_MoveToGoal(*args, **kwargs))
        return goal.get_result()

    def set_position(self, position, unit='m'):
        self.position = np.array(position) * UNITS[unit]

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
    
    # When C3 gets replaced, these may go away
    def as_MoveToGoal(self, linear=[0, 0, 0], angular=[0, 0, 0], **kwargs):
        return MoveToGoal(
            header=Header(
                frame_id=self.frame_id,
            ),
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
class PoseEditor(object):
    
    def from_Odometry(cls, msg):
        return cls.from_Pose(msg.header.frame_id, msg.pose.pose)

    def from_PoseTwistStamped(cls, msg):
        return cls.from_Pose(msg.header.frame_id, msg.posetwist.pose)

    def from_Pose(cls, frame_id, msg):
        return cls(frame_id, rosmsg_to_numpy(msg.position), rosmsg_to_numpy(msg.orientation))

    def __init__(self, ac, frame_id, position, orientation, s_position=None, s_orientation=None):
        self.frame_id = frame_id

        self.position = position
        self.orientation = orientation

        self.s_position = s_position
        self.s_orientation = s_orientation

        self.action_client = ac

    # def __repr__(self):
    #     return "{} - p: {}, q: {}".format(self.frame_id, self.position, self.orientation)

    @property
    def _rot(self):
        return transformations.quaternion_matrix(self.orientation)[:3, :3]

    @property
    def forward_vector(self):
        return self._rot.dot([1, 0, 0])

    @property
    def left_vector(self):
        return self._rot.dot([0, 1, 0])

    @property
    def body_up_vector(self):
        return self._rot.dot(UP)

    def start_point(self, target=None):
        if target is None:
            return
        

    # def from_general_message(self, msg):
    #     if isinstance(msg, geometry_msgs.msg._Pose.Pose):
    #         self.

    def set_position(self, position):
        self.position = position
        return self

    def depth(self, depth):
        return self.set_position([self.position[0], self.position[1], -depth])

    def relative(self, rel_pos):
        return self.set_position(self.position + self._rot.dot(rel_pos))

    def strafe_relative(self, rel_pos_2d):
        rel_pos_3d = np.append(rel_pos_2d, 0.0)
        return self.set_position(self.position + self._rot.dot(rel_pos_3d))

    def forward(self, distance):
        return self.relative([+distance, 0, 0])

    def backward(self, distance):
        return self.relative([-distance, 0, 0])

    def left(self, distance):
        return self.relative([0, +distance, 0])

    def right(self, distance):
        return self.relative([0, -distance, 0])

    def body_up(self, distance):
        return self.relative([0, 0, +distance])

    def body_down(self, distance):
        return self.relative([0, 0, -distance])

    def absolute(self, abs_pos):
        return type(self)(self.frame_id, self.position + abs_pos, self.orientation)

    def east(self, distance):
        return self.absolute([+distance, 0, 0])

    def west(self, distance):
        return self.absolute([-distance, 0, 0])

    def north(self, distance):
        return self.absolute([0, +distance, 0])

    def south(self, distance):
        return self.absolute([0, -distance, 0])

    def up(self, distance):
        return self.absolute([0, 0, +distance])

    def down(self, distance):
        return self.absolute([0, 0, -distance])

    # Orientation
    def set_orientation(self, orientation):
        return type(self)(self.frame_id, self.position, orientation)

    def look_at_rel(self, rel_point):
        return self.set_orientation(look_at(rel_point))

    def look_at(self, point):
        return self.look_at_rel(point - self.position)

    def look_at_rel_without_pitching(self, rel_point):
        return self.set_orientation(look_at_without_pitching(rel_point))

    def look_at_without_pitching(self, point):
        return self.look_at_rel_without_pitching(point - self.position)

    def point_vec_towards(self, body_vec, towards_point):
        return self.point_vec_towards_rel(body_vec, towards_point - self.pos)

    def point_vec_towards_rel(self, body_vec, towards_rel_point):
        return self.set_orientation(triad((towards_rel_point, UP), (body_vec, UP)))

    def turn_vec_towards(self, body_vec, towards_point):
        return self.turn_vec_towards_rel(body_vec, towards_point - self.pos)

    def turn_vec_towards_rel(self, body_vec, towards_rel_point):
        return self.set_orientation(triad((UP, towards_rel_point), (UP, body_vec)))

    def yaw_left(self, angle):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle, UP),
            self.orientation,
        ))

    def yaw_right(self, angle):
        return self.yaw_left(-angle)

    def yaw_left_deg(self, angle_degrees):
        return self.yaw_left(np.radians(angle_degrees))

    def yaw_right_deg(self, angle_degrees):
        return self.yaw_right(np.radians(angle_degrees))

    turn_left = yaw_left
    turn_right = yaw_right
    turn_left_deg = yaw_left_deg
    turn_right_deg = yaw_right_deg

    def heading(self, heading):
        return self.set_orientation(
            transformations.quaternion_about_axis(heading, UP)
        )

    def heading_deg(self, heading_deg):
        return self.heading(np.radians(heading_deg))

    def roll_right(self, angle):
        return self.set_orientation(transformations.quaternion_multiply(
            self.orientation,
            transformations.quaternion_about_axis(angle, [1, 0, 0]),
        ))

    def roll_left(self, angle):
        return self.roll_right(-angle)

    def roll_right_deg(self, angle_degrees):
        return self.roll_right(np.radians(angle_degrees))

    def roll_left_deg(self, angle_degrees):
        return self.roll_left(np.radians(angle_degrees))

    def zero_roll(self):
        return self.set_orientation(look_at(self.forward_vector))

    def pitch_down(self, angle):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle, self.zero_roll().left_vector),
            self.orientation,
        ))

    def pitch_up(self, angle):
        return self.pitch_down(-angle)

    def pitch_down_deg(self, angle_degrees):
        return self.pitch_down(np.radians(angle_degrees))

    def pitch_up_deg(self, angle_degrees):
        return self.pitch_up(np.radians(angle_degrees))

    def zero_roll_and_pitch(self):
        return self.set_orientation(look_at_without_pitching(self.forward_vector))

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

    def as_PoseTwistStamped(self, linear=[0, 0, 0], angular=[0, 0, 0]):
        return PoseTwistStamped(
            header=Header(
                frame_id=self.frame_id,
            ),
            posetwist=self.as_PoseTwist(linear, angular),
        )

    def as_MoveToGoal(self, linear=[0, 0, 0], angular=[0, 0, 0], **kwargs):
        return MoveToGoal(
            header=Header(
                frame_id=self.frame_id,
            ),
            posetwist=self.as_PoseTwist(linear, angular),
            **kwargs
        )