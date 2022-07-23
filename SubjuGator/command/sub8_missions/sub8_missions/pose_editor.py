from __future__ import annotations

import warnings
from typing import Optional, Sequence

import numpy as np
import rospy
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose as Pose
from geometry_msgs.msg import Quaternion, Twist, Vector3
from mil_msgs.msg import MoveToGoal, PoseTwist, PoseTwistStamped
from mil_ros_tools import rosmsg_to_numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf import transformations

UP = np.array([0.0, 0.0, 1.0], np.float64)
EAST, NORTH, WEST, SOUTH = (
    transformations.quaternion_about_axis(np.pi / 2 * i, UP) for i in range(4)
)


def normalized(x: np.ndarray) -> np.ndarray:
    x = np.array(x)
    if max(map(abs, x)) == 0:
        warnings.warn("Normalizing zero-length vector to random unit vector")
        x = np.random.standard_normal(x.shape)
    x = x / max(map(abs, x))
    x = x / np.linalg.norm(x)
    return x


def get_perpendicular(a: np.ndarray, b: np.ndarray | None = None) -> np.ndarray:
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


def quat_to_rotvec(q: np.ndarray):
    if q[3] < 0:
        q = -q
    q = transformations.unit_vector(q)
    angle = np.arccos(q[3]) * 2
    axis = normalized(q[0:3])
    return axis * angle


def rotvec_to_quat(rotvec: np.ndarray) -> np.ndarray:
    return transformations.quaternion_about_axis(np.linalg.norm(rotvec), rotvec)


def triad(xxx_todo_changeme, xxx_todo_changeme1) -> np.ndarray:
    # returns quaternion that rotates b1 to a1 and b2 near a2
    # can get orientation by passing in (global, local)
    (a1, a2) = xxx_todo_changeme
    (b1, b2) = xxx_todo_changeme1
    aa = get_perpendicular(a1, a2)
    A = np.array([normalized(a1), aa, normalized(np.cross(a1, aa))])
    bb = get_perpendicular(b1, b2)
    B = np.array([normalized(b1), bb, normalized(np.cross(b1, bb))])
    rot = A.T.dot(B)
    return transformations.quaternion_from_matrix(
        [(a, b, c, 0) for a, b, c in rot] + [(0, 0, 0, 1)]
    )


def test_triad():
    q = transformations.random_quaternion()

    a = np.random.standard_normal(3)
    b = np.random.standard_normal(3)

    m = transformations.quaternion_matrix(q)[:3, :3]
    q_ = triad((m.dot(a), m.dot(b)), (a, b))

    assert (
        np.linalg.norm(
            quat_to_rotvec(
                transformations.quaternion_multiply(
                    q,
                    transformations.quaternion_inverse(q_),
                )
            )
        )
        < 1e-6
    )


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
            return rospy.wait_for_message(topic, topic_type, 0.5)
        except rospy.exceptions.ROSException as e:
            if "timeout" not in e.message:
                raise
            print(topic, "wait_for_message timed out!")


class PoseEditor:
    @classmethod
    def from_Odometry_topic(cls, topic: str = "/odom") -> PoseEditor:
        return cls.from_Odometry(safe_wait_for_message(topic, Odometry))

    @classmethod
    def from_Odometry(cls, msg: Odometry) -> PoseEditor:
        return cls.from_Pose(msg.header.frame_id, msg.pose.pose)

    @classmethod
    def from_PoseTwistStamped_topic(cls, topic: str) -> PoseEditor:
        return cls.from_PoseTwistStamped(safe_wait_for_message(topic, PoseTwistStamped))

    @classmethod
    def from_PoseTwistStamped(cls, msg: PoseTwistStamped) -> PoseEditor:
        return cls.from_Pose(msg.header.frame_id, msg.posetwist.pose)

    @classmethod
    def from_Pose(cls, frame_id: str, msg: Pose) -> PoseEditor:
        return cls(
            frame_id, rosmsg_to_numpy(msg.position), rosmsg_to_numpy(msg.orientation)
        )

    def __init__(self, frame_id: str, position, orientation):
        self.frame_id = frame_id
        self.position = position
        self.orientation = orientation

    def __str__(self):
        return f"p: {self.position}, q: {self.orientation}"

    @property
    def _rot(self) -> np.ndarray:
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

    # Position
    def set_position(self, position):
        return type(self)(self.frame_id, position, self.orientation)

    def depth(self, depth):
        return self.set_position([self.position[0], self.position[1], -depth])

    def relative(self, rel_pos):
        return self.set_position(self.position + self._rot.dot(rel_pos))

    def strafe_relative(self, rel_pos_2d):
        rel_pos_3d = np.append(rel_pos_2d, 0.0)
        return self.set_position(self.position + self._rot.dot(rel_pos_3d))

    def relative_depth(self, rel_pos):
        old_z = self.position[2]
        pose = self.relative(rel_pos)
        pose.position[2] = old_z
        return pose

    def forward(self, distance: float) -> PoseEditor:
        return self.relative([+distance, 0, 0])

    def backward(self, distance: float) -> PoseEditor:
        return self.relative([-distance, 0, 0])

    def left(self, distance: float) -> PoseEditor:
        return self.relative([0, +distance, 0])

    def right(self, distance: float) -> PoseEditor:
        return self.relative([0, -distance, 0])

    def strafe_forward(self, distance: float):
        return self.relative_depth([+distance, 0, 0])

    def strafe_backward(self, distance: float):
        return self.relative_depth([-distance, 0, 0])

    def strafe_left(self, distance: float):
        return self.relative_depth([0, +distance, 0])

    def strafe_right(self, distance: float):
        return self.relative_depth([0, -distance, 0])

    def body_up(self, distance: float):
        return self.relative([0, 0, +distance])

    def body_down(self, distance: float):
        return self.relative([0, 0, -distance])

    def absolute(self, abs_pos):
        return type(self)(self.frame_id, self.position + abs_pos, self.orientation)

    def east(self, distance: float):
        return self.absolute([+distance, 0, 0])

    def west(self, distance: float):
        return self.absolute([-distance, 0, 0])

    def north(self, distance: float):
        return self.absolute([0, +distance, 0])

    def south(self, distance: float):
        return self.absolute([0, -distance, 0])

    def up(self, distance: float):
        return self.absolute([0, 0, +distance])

    def down(self, distance: float):
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

    def turn_vec_towards_rel(self, body_vec, towards_rel_point) -> PoseEditor:
        return self.set_orientation(triad((UP, towards_rel_point), (UP, body_vec)))

    def yaw_left(self, angle: float) -> PoseEditor:
        return self.set_orientation(
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(angle, UP),
                self.orientation,
            )
        )

    def yaw_right(self, angle: float) -> PoseEditor:
        return self.yaw_left(-angle)

    def yaw_left_deg(self, angle_degrees: float) -> PoseEditor:
        return self.yaw_left(np.radians(angle_degrees))

    def yaw_right_deg(self, angle_degrees: float) -> PoseEditor:
        return self.yaw_right(np.radians(angle_degrees))

    turn_left = yaw_left
    turn_right = yaw_right
    turn_left_deg = yaw_left_deg
    turn_right_deg = yaw_right_deg

    def heading(self, heading: float) -> PoseEditor:
        return self.set_orientation(transformations.quaternion_about_axis(heading, UP))

    def heading_deg(self, heading_deg: float) -> PoseEditor:
        return self.heading(np.radians(heading_deg))

    def roll_right(self, angle: float) -> PoseEditor:
        return self.set_orientation(
            transformations.quaternion_multiply(
                self.orientation,
                transformations.quaternion_about_axis(angle, [1, 0, 0]),
            )
        )

    def roll_left(self, angle: float) -> PoseEditor:
        return self.roll_right(-angle)

    def roll_right_deg(self, angle_degrees: float) -> PoseEditor:
        return self.roll_right(np.radians(angle_degrees))

    def roll_left_deg(self, angle_degrees: float) -> PoseEditor:
        return self.roll_left(np.radians(angle_degrees))

    def zero_roll(self) -> PoseEditor:
        return self.set_orientation(look_at(self.forward_vector))

    def pitch_down(self, angle: float) -> PoseEditor:
        return self.set_orientation(
            transformations.quaternion_multiply(
                transformations.quaternion_about_axis(
                    angle, self.zero_roll().left_vector
                ),
                self.orientation,
            )
        )

    def pitch_up(self, angle: float) -> PoseEditor:
        return self.pitch_down(-angle)

    def pitch_down_deg(self, angle_degrees: float) -> PoseEditor:
        return self.pitch_down(np.radians(angle_degrees))

    def pitch_up_deg(self, angle_degrees: float) -> PoseEditor:
        return self.pitch_up(np.radians(angle_degrees))

    def zero_roll_and_pitch(self) -> PoseEditor:
        return self.set_orientation(look_at_without_pitching(self.forward_vector))

    def as_Pose(self) -> Pose:
        return Pose(
            position=Point(*self.position),
            orientation=Quaternion(*self.orientation),
        )

    def as_PoseTwist(
        self, linear: Sequence[int] = [0, 0, 0], angular: Sequence[int] = [0, 0, 0]
    ):
        return PoseTwist(
            pose=self.as_Pose(),
            twist=Twist(
                linear=Vector3(*linear),
                angular=Vector3(*angular),
            ),
        )

    def as_PoseTwistStamped(
        self, linear: Sequence[int] = [0, 0, 0], angular: Sequence[int] = [0, 0, 0]
    ) -> PoseTwistStamped:
        return PoseTwistStamped(
            header=Header(
                frame_id=self.frame_id,
            ),
            posetwist=self.as_PoseTwist(linear, angular),
        )

    def as_MoveToGoal(
        self,
        linear: Sequence[int] = [0, 0, 0],
        angular: Sequence[int] = [0, 0, 0],
        **kwargs,
    ) -> MoveToGoal:
        return MoveToGoal(
            header=Header(
                frame_id=self.frame_id,
            ),
            posetwist=self.as_PoseTwist(linear, angular),
            **kwargs,
        )

    # allow implicit usage in place of a PoseTwistStamped
    @property
    def header(self) -> Header:
        return Header(
            frame_id=self.frame_id,
        )

    @property
    def posetwist(self) -> PoseTwist:
        return self.as_PoseTwist()

    # and in place of a MoveToGoal

    @property
    def speed(self) -> int:
        return 0

    @property
    def uncoordinated(self) -> bool:
        return False

    @property
    def linear_tolerance(self) -> int:
        return 0

    @property
    def angular_tolerance(self) -> int:
        return 0
