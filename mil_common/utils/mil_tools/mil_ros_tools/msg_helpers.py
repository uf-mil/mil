import numpy as np
from tf import transformations
import geometry_msgs.msg as geometry_msgs
import std_msgs.msg as std_msgs
import nav_msgs.msg as nav_msgs
from mil_msgs.msg import Point2D, PoseTwist
import rospy
from typing import Tuple, List


def rosmsg_to_numpy(rosmsg, keys=None):
    """
    ===============
    NOTE: This method should be removed. Please see issue #469 on the GitHub
    repository.
    ===============
    Convert an arbitrary ROS msg to a numpy array
    With no additional arguments, it will by default handle:
        Point2D, Point3D, Vector3D, Quaternion and any lists of these (like Polygon)

    Ex:
        quat = Quaternion(1.0, 0.0, 0.0, 0.0)
        quat is not a vector, you have quat.x, quat.y,... and you can't do math on that

        But wait, there's hope!
            rosmsg_to_numpy(quat) -> array([1.0, 0.0, 0.0, 0.0])

        Yielding a vector, which you can do math on!

        Further, imagine a bounding box message, BB, with properties BB.x, BB.h, BB.y, and BB.w

            rosmsg_to_numpy(BB, ['x', 'h', 'y', 'w']) -> array([BB.x, BB.h, BB.y, BB.w])

            or...
            rosmsg_to_numpy(some_Pose2D, ['x', 'y', 'yaw']) = array([x, y, yaw])

    Note:
        - This function is designed to handle the most common use cases (vectors, points and quaternions)
            without requiring any additional arguments.
    """

    # Recurse for lists like geometry_msgs/Polygon, Pointclou
    if type(rosmsg) == list:
        output_array = []
        for item in rosmsg:
            output_array.append(rosmsg_to_numpy(item, keys=keys))
        return np.array(output_array).astype(np.float32)

    if keys is None:
        keys = ["x", "y", "z", "w"]
        output_array = []
        for key in keys:
            # This is not necessarily the fastest way to do this
            if hasattr(rosmsg, key):
                output_array.append(getattr(rosmsg, key))
            else:
                break
        assert (
            len(output_array) != 0
        ), "Input type {} has none of these attributes {}.".format(
            type(rosmsg).__name__, keys
        )
        return np.array(output_array).astype(np.float32)

    else:
        output_array = np.zeros(len(keys), np.float32)
        for n, key in enumerate(keys):
            output_array[n] = getattr(rosmsg, key)

        return output_array


def pose_to_numpy(pose: geometry_msgs.Pose) -> Tuple[float, float]:
    """
    Turns a :class:`Pose` message into a tuple of position and orientation.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        pose (Pose): The pose message.

    Returns:
        Tuple[float, float]: The tuple of position and orientation.
    """
    # TODO Add unit tests
    position = rosmsg_to_numpy(pose.position)
    orientation = rosmsg_to_numpy(pose.orientation)
    return position, orientation


def twist_to_numpy(twist: geometry_msgs.Twist):
    """
    Turns a :class:`Twist` message into a tuple of linear and angular speeds.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        twist (Twist): The twist message.

    Returns:
        Tuple[List[float], List[float]]: The tuple of linear and angular speeds.
    """
    # TODO Add unit tests
    linear = rosmsg_to_numpy(twist.linear)
    angular = rosmsg_to_numpy(twist.angular)
    return linear, angular


def posetwist_to_numpy(posetwist: PoseTwist):
    """
    Turns a :class:`PoseTwist` message into pose position, pose orientation, twist linear, and twist angular.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        twist (PoseTwist): The pose + twist message.

    Returns:
        Tuple[Tuple[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray]]: The tuple of linear and angular speeds.
    """
    pose = pose_to_numpy(posetwist.pose)
    twist = twist_to_numpy(posetwist.twist)
    return pose, twist


def odometry_to_numpy(odom: nav_msgs.Odometry):
    """
    Turns a :class:`Odometry` message into its pose, twist, pose covariance,
    and twist covariance.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        odom (Odometry): The odometry message.

    Returns:
        Tuple[Tuple[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray], 
            np.ndarray, np.ndarray]: The tuple of pose, twist, pose covariance,
            and twist covariance.
    """
    # TODO Add unit tests
    pose = pose_to_numpy(odom.pose.pose)
    pose_covariance = np.array(odom.pose.covariance).reshape(6, 6)

    twist = twist_to_numpy(odom.twist.twist)
    twist_covariance = np.array(odom.twist.covariance).reshape(6, 6)

    return pose, twist, pose_covariance, twist_covariance


def wrench_to_numpy(wrench: geometry_msgs.Wrench):
    """
    Turns a :class:`Wrench` message into its force and torque, represented as
    numpy arrays.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        wrench (Wrench): The wrench message.

    Returns:
        Tuple[np.ndarray, np.ndarray]: The tuple of force and torque.
    """
    force = rosmsg_to_numpy(wrench.force)
    torque = rosmsg_to_numpy(wrench.torque)
    return force, torque


def numpy_to_point(vector: np.ndarray) -> geometry_msgs.Point:
    """
    Turns a List[:class:`float`] into a :class:`Point` message.

    Args:
        vector (np.ndarray): The vector to convert

    Returns:
        geometry_msgs.Point: The constructed message.
    """
    np_vector = np.array(vector)
    if np_vector.size == 2:
        np_vector = np.append(np_vector, 0)  # Assume user is give 2d point

    return geometry_msgs.Point(*np_vector)


def numpy_to_point2d(vector: np.ndarray) -> Point2D: 
    """
    Turns a :class:`np.ndarray` into a :class:`Point2D` message.

    Args:
        vector (np.ndarray): The vector to convert. Should have two values, the
            first of which represents x and the other y.

    Returns:
        Point2D: The constructed message.
    """
    np_vector = np.array(vector)
    return Point2D(*np_vector)


def numpy_to_quaternion(np_quaternion: np.ndarray) -> geometry_msgs.Quaternion:
    """
    Turns a List[:class:`float`] into a :class:`Quaternion` message.

    Args:
        np_quaternion (np.ndarray): The vector to convert. Should have four values,
            representing ``x``, ``y``, ``z``, and ``w``.

    Returns:
        Quaternion: The constructed message.
    """
    return geometry_msgs.Quaternion(*np_quaternion)


def numpy_to_twist(linear_vel: np.ndarray, angular_vel: np.ndarray) -> geometry_msgs.Twist:
    """
    Turns two :class:`np.ndarray`s into a :class:`Twist` message.

    Args:
        linear_vel (np.ndarray): The vector to convert. Values should represent
            the individual components of ``x``, ``y``, and ``z``.
        angular_vel (np.ndarray): The vector to convert. Values should represent
            the individual components of ``x``, ``y``, and ``z``.

    Returns:
        Twist: The constructed message.
    """
    # TODO Add unit tests
    return geometry_msgs.Twist(
        linear=geometry_msgs.Vector3(*linear_vel),
        angular=geometry_msgs.Vector3(*angular_vel),
    )


def numpy_to_wrench(forcetorque: np.ndarray):
    """
    Turns a np.ndarray into a :class:`Wrench` message.

    Args:
        forcetorque (np.ndarray): The vector to convert. Values should represent
            the individual components of ``force.x``, ``force.y``, ``force.z``,
            ``torque.x``, ``torque.y``, and ``torque.z``.

    Returns:
        Wrench: The constructed message.
    """
    return geometry_msgs.Wrench(
        force=geometry_msgs.Vector3(*forcetorque[:3]),
        torque=geometry_msgs.Vector3(*forcetorque[3:]),
    )


def numpy_matrix_to_quaternion(np_matrix: np.ndarray):
    """
    Given a 3x3 rotation matrix, convert to a quaternion, and return the quaternion
    as a ROS message.

    Args:
        np_matrix (np.ndarray): The 3x3 rotation matrix.

    Returns:
        Quaternion: The constructed message.
    """
    assert np_matrix.shape == (3, 3), "Must submit 3x3 rotation matrix"
    pose_mat = np.eye(4)
    pose_mat[:3, :3] = np_matrix
    np_quaternion = transformations.quaternion_from_matrix(pose_mat)
    return geometry_msgs.Quaternion(*np_quaternion)


def numpy_pair_to_pose(np_translation: np.ndarray, np_rotation_matrix: np.ndarray) -> geometry_msgs.Pose:
    """
    Convert a rotation matrix and point pair to a Pose message.

    Args:
        np_translation (np.ndarray): An array of 2 or 3 points representing the
            object's position.
        np_rotation_matrix (np.ndarray): A 3x3 rotation matrix.

    Returns:
        geometry_msgs.Pose: The Pose message with the position and orientation.
    """
    orientation = numpy_matrix_to_quaternion(np_rotation_matrix)
    position = numpy_to_point(np_translation)
    return geometry_msgs.Pose(position=position, orientation=orientation)


def numpy_quat_pair_to_pose(np_translation: np.ndarray, np_quaternion: np.ndarray) -> geometry_msgs.Pose:
    """
    Convert a quaternion array and point array pair to a Pose message.

    Args:
        np_translation (np.ndarray): An array of 2 or 3 points representing the
            object's position.
        np_quaternion (np.ndarray): An array with 4 points, representing ``x``,
            ``y``, ``z``, and ``w`` of a quaternion describing the object.

    Returns:
        geometry_msgs.Pose: The Pose message with the position and orientation.
    """
    orientation = numpy_to_quaternion(np_quaternion)
    position = numpy_to_point(np_translation)
    return geometry_msgs.Pose(position=position, orientation=orientation)


def numpy_to_points(points):
    ret = []
    for point in points:
        ret.append(numpy_to_point(point))
    return ret


def numpy_to_polygon(polygon):
    points = numpy_to_points(polygon)
    return geometry_msgs.Polygon(points=points)


def numpy_to_vector3(vec):
    assert len(vec) == 3
    return geometry_msgs.Vector3(*vec)


def numpy_to_pose2D(pose):
    return geometry_msgs.Pose2D(*pose)


def numpy_to_colorRGBA(color):
    return std_msgs.ColorRGBA(*color)


def make_header(frame="/body", stamp=None):
    if stamp is None:
        try:
            stamp = rospy.Time.now()
        except rospy.ROSInitException:
            stamp = rospy.Time(0)

    header = std_msgs.Header(stamp=stamp, frame_id=frame)
    return header


def make_wrench_stamped(force, torque, frame="/body"):
    """
    Make a WrenchStamped message without all the fuss
    Frame defaults to body
    """
    wrench = geometry_msgs.WrenchStamped(
        header=make_header(frame),
        wrench=geometry_msgs.Wrench(
            force=geometry_msgs.Vector3(*force), torque=geometry_msgs.Vector3(*torque)
        ),
    )
    return wrench


def make_pose_stamped(position, orientation, frame="/body"):
    wrench = geometry_msgs.WrenchStamped(
        header=make_header(frame),
        pose=geometry_msgs.Pose(
            force=geometry_msgs.Vector3(*position),
            orientation=geometry_msgs.Quaternion(*orientation),
        ),
    )
    return wrench


def odom_sub(topic, callback):
    def wrapped_callback(*args):
        msg = args[-1]
        callback(odometry_to_numpy(msg))

    return rospy.Subscriber(topic, nav_msgs.Odometry, wrapped_callback, queue_size=1)


def ros_to_np_3D(msg):
    xyz_array = np.array(([msg.x, msg.y, msg.z]))
    return xyz_array
