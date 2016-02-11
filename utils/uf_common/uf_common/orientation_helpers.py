from __future__ import division

import math
import warnings

import numpy

import roslib
roslib.load_manifest('uf_common')
import rospy
from tf import transformations
from nav_msgs.msg import Odometry
from uf_common.msg import PoseTwistStamped, PoseTwist, MoveToGoal
from std_msgs.msg import Header
from geometry_msgs.msg import Pose as Pose, Quaternion, Point, Vector3, Twist


def normalized(x):
    x = numpy.array(x)
    if max(map(abs, x)) == 0:
        warnings.warn('Normalizing zero-length vector to random unit vector')
        x = numpy.random.standard_normal(x.shape)
    x = x / max(map(abs, x))
    x = x / numpy.linalg.norm(x)
    return x

def get_perpendicular(a, b=None):
    a = numpy.array(a)
    if max(map(abs, a)) == 0:
        if b is not None: return get_perpendicular(b)
        return normalized(numpy.random.standard_normal(a.shape))
    if b is None:
        b = numpy.random.standard_normal(a.shape)
    b = numpy.array(b)
    x = numpy.cross(a, b)
    if max(map(abs, x)) == 0:
        return get_perpendicular(a)
    return normalized(x)


def quat_to_rotvec(q):
    if q[3] < 0:
        q = -q
    q = transformations.unit_vector(q)
    angle = math.acos(q[3])*2
    axis = normalized(q[0:3])
    return axis * angle

def rotvec_to_quat(rotvec):
    return transformations.quaternion_about_axis(numpy.linalg.norm(rotvec), rotvec)


xyz_array = lambda o: numpy.array([o.x, o.y, o.z])
xyzw_array = lambda o: numpy.array([o.x, o.y, o.z, o.w])


def triad((a1, a2), (b1, b2)):
    # returns quaternion that rotates b1 to a1 and b2 near a2
    # can get orientation by passing in (global, local)
    aa = get_perpendicular(a1, a2)
    A = numpy.array([normalized(a1), aa, normalized(numpy.cross(a1, aa))])
    bb = get_perpendicular(b1, b2)
    B = numpy.array([normalized(b1), bb, normalized(numpy.cross(b1, bb))])
    rot = A.T.dot(B)
    return transformations.quaternion_from_matrix(
        [(a,b,c,0) for a,b,c in rot]+
        [(0,0,0,1)])

def test_triad():
    q = transformations.random_quaternion()
    
    a = numpy.random.standard_normal(3)
    b = numpy.random.standard_normal(3)
    
    m = transformations.quaternion_matrix(q)[:3, :3]
    q_ = triad((m.dot(a), m.dot(b)), (a, b))
    
    assert numpy.linalg.norm(quat_to_rotvec(
        transformations.quaternion_multiply(
            q,
            transformations.quaternion_inverse(q_),
        )
    )) < 1e-6
test_triad()


def look_at(forward, upish=[0, 0, 1]):
    # assumes standard forward-left-up body coordinate system
    return triad((forward, upish), ([1, 0, 0], [0, 0, 1]))
lookat = look_at # deprecated

def look_at_without_pitching(forwardish, up=[0, 0, 1]):
    # assumes standard forward-left-up body coordinate system
    return triad((up, forwardish), ([0, 0, 1], [1, 0, 0]))
lookat_without_pitching = look_at_without_pitching # deprecated

def look_at_camera(forward, upish=[0, 0, 1]):
    # assumes camera right-down-forward coordinate system
    return triad((forward, upish), ([0, 0, 1], [0, -1, 0]))
lookat_camera = look_at_camera # deprecated


EAST, NORTH, WEST, SOUTH = [transformations.quaternion_about_axis(math.pi/2*i, [0, 0, 1]) for i in xrange(4)]

def safe_wait_for_message(topic, topic_type):
    while True:
        try:
            return rospy.wait_for_message(topic, topic_type, .5)
        except rospy.exceptions.ROSException, e:
            if 'timeout' not in e.message: raise
            print topic, 'wait_for_message timed out!'
            

class PoseEditor(object):
    @classmethod
    def from_Odometry_topic(cls, topic='/odom'):
        return cls.from_Odometry(safe_wait_for_message(topic, Odometry))
    
    @classmethod
    def from_Odometry(cls, msg):
        return cls.from_Pose(msg.header.frame_id, msg.pose.pose)
    
    @classmethod
    def from_PoseTwistStamped_topic(cls, topic):
        return cls.from_PoseTwistStamped(safe_wait_for_message(topic, PoseTwistStamped))
    
    @classmethod
    def from_PoseTwistStamped(cls, msg):
        return cls.from_Pose(msg.header.frame_id, msg.posetwist.pose)
    
    @classmethod
    def from_Pose(cls, frame_id, msg):
        return cls(frame_id, xyz_array(msg.position), xyzw_array(msg.orientation))
    
    def __init__(self, frame_id, position, orientation):
        self.frame_id = frame_id
        self.position = position
        self.orientation = orientation
    
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
        return self._rot.dot([0, 0, 1])
    
    
    def set_position(self, position):
        return type(self)(self.frame_id, position, self.orientation)
    
    def height(self, height):
        return self.set_position([self.position[0], self.position[1], height])
    def depth(self, depth):
        return self.set_position([self.position[0], self.position[1], -depth])
    
    def relative(self, rel_pos):
        return self.set_position(self.position + self._rot.dot(rel_pos))
    
    def   forward(self, distance): return self.relative([ distance, 0, 0])
    def  backward(self, distance): return self.relative([-distance, 0, 0])
    def      left(self, distance): return self.relative([0,  distance, 0])
    def     right(self, distance): return self.relative([0, -distance, 0])
    def   body_up(self, distance): return self.relative([0, 0,  distance])
    def body_down(self, distance): return self.relative([0, 0, -distance])
    
    def absolute(self, abs_pos):
        return type(self)(self.frame_id, self.position + abs_pos, self.orientation)
    
    def  east(self, distance): return self.absolute([ distance, 0, 0])
    def  west(self, distance): return self.absolute([-distance, 0, 0])
    def north(self, distance): return self.absolute([0,  distance, 0])
    def south(self, distance): return self.absolute([0, -distance, 0])
    def    up(self, distance): return self.absolute([0, 0,  distance])
    def  down(self, distance): return self.absolute([0, 0, -distance])
    
    
    def set_orientation(self, orientation):
        return type(self)(self.frame_id, self.position, orientation)
    
    def look_at_rel(self, rel_point): return self.set_orientation(look_at(rel_point))
    def look_at(self, point): return self.look_at_rel(point - self.position)
    def look_at_rel_without_pitching(self, rel_point): return self.set_orientation(look_at_without_pitching(rel_point))
    def look_at_without_pitching(self, point): return self.look_at_rel_without_pitching(point - self.position)
    
    def point_vec_towards(self, body_vec, towards_point): return self.point_vec_towards_rel(body_vec, towards_point - self.pos)
    def point_vec_towards_rel(self, body_vec, towards_rel_point): return self.set_orientation(triad((towards_rel_point, [0, 0, 1]), (body_vec, [0, 0, 1])))
    def turn_vec_towards(self, body_vec, towards_point): return self.turn_vec_towards_rel(body_vec, towards_point - self.pos)
    def turn_vec_towards_rel(self, body_vec, towards_rel_point): return self.set_orientation(triad(([0, 0, 1], towards_rel_point), ([0, 0, 1], body_vec)))
    
    def yaw_left(self, angle):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle, [0, 0, 1]),
            self.orientation,
        ))
    def yaw_right(self, angle): return self.yaw_left(-angle)
    def yaw_left_deg(self, angle_degrees): return self.yaw_left(math.radians(angle_degrees))
    def yaw_right_deg(self, angle_degrees): return self.yaw_right(math.radians(angle_degrees))
    turn_left = yaw_left
    turn_right = yaw_right
    turn_left_deg = yaw_left_deg
    turn_right_deg = yaw_right_deg
    def heading(self, heading):
        return self.set_orientation(
            transformations.quaternion_about_axis(heading, [0, 0, 1])
        )
    def heading_deg(self, heading_deg):
        return self.heading(math.radians(heading_deg))
    
    def roll_right(self, angle):
        return self.set_orientation(transformations.quaternion_multiply(
            self.orientation,
            transformations.quaternion_about_axis(angle, [1, 0, 0]),
        ))
    def roll_left(self, angle): return self.roll_right(-angle)
    def roll_right_deg(self, angle_degrees): return self.roll_right(math.radians(angle_degrees))
    def roll_left_deg(self, angle_degrees): return self.roll_left(math.radians(angle_degrees))
    def zero_roll(self):
        return self.set_orientation(lookat(self.forward_vector))
    
    
    def pitch_down(self, angle):
        return self.set_orientation(transformations.quaternion_multiply(
            transformations.quaternion_about_axis(angle, self.zero_roll().left_vector),
            self.orientation,
        ))
    def pitch_up(self, angle): return self.pitch_down(-angle)
    def pitch_down_deg(self, angle_degrees): return self.pitch_down(math.radians(angle_degrees))
    def pitch_up_deg(self, angle_degrees): return self.pitch_up(math.radians(angle_degrees))
    def zero_roll_and_pitch(self):
        return self.set_orientation(lookat_without_pitching(self.forward_vector))
    
    
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
        **kwargs)
    
    # allow implicit usage in place of a PoseTwistStamped
    @property
    def header(self):
        return Header(
            frame_id=self.frame_id,
        )
    @property
    def posetwist(self):
        return self.as_PoseTwist()
    # and in place of a MoveToGoal
    @property
    def speed(self):
        return 0
    @property
    def uncoordinated(self):
        return False
    @property
    def linear_tolerance(self):
        return 0
    @property
    def angular_tolerance(self):
        return 0
