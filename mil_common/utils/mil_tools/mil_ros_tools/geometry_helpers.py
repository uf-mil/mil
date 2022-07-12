from __future__ import division

from typing import List

import numpy as np
import numpy.typing as npt
import tf.transformations as trans
from geometry_msgs.msg import Quaternion

from .msg_helpers import numpy_quat_pair_to_pose


def rotate_vect_by_quat(v: List[float], q: List[float]) -> np.ndarray:
    """
    Rotates a vector by a quaterion using ``v' = q*vq``. Both ``v`` and ``q``
    should be a list of 4 floats representing [x, y, z, w].

    Args:
        v (List[float]): The vector to perform the multiplication on.
        q (List[float]): The quaternion to mulitply the vector by.

    Returns:
        np.ndarray: The numpy array representing the new calculated quaternion.
    """
    cq = np.array([-q[0], -q[1], -q[2], q[3]])
    cq_v = trans.quaternion_multiply(cq, v)
    v = trans.quaternion_multiply(cq_v, q)
    v[
        1:
    ] *= (
        -1
    )  # Only seemed to work when I flipped this around, is there a problem with the math here?
    return np.array(v)[:3]


def make_rotation(vector_a: List[float], vector_b: List[float]) -> npt.NDArray:
    """
    Determine a 3D rotation that rotates A onto B. In other words, we want a
    matrix R that aligns A with B.

    .. code-block:: python3

        >>> R = make_rotation(a, b)
        >>> p = R.dot(a)
        >>> np.cross(p, a)
        array([0.0, 0.0, 0.0])

    """
    # [1] Calculate Rotation Matrix to align Vector A to Vector B in 3d?
    #     http://math.stackexchange.com/questions/180418
    # [2] N. Ho, Finding Optimal Rotation...Between Corresponding 3D Points
    #     http://nghiaho.com/?page_id=671
    unit_a = normalize(vector_a)
    unit_b = normalize(vector_b)

    v = np.cross(unit_a, unit_b)
    s = np.linalg.norm(v)

    c = np.dot(unit_a, unit_b)

    skew_cross = skew_symmetric_cross(v)
    skew_squared = np.linalg.matrix_power(skew_cross, 2)

    if np.isclose(c, 1.0, atol=1e-4):
        R = np.eye(3)
        return R
    elif np.isclose(c, -1.0, atol=1e-4):
        R = np.eye(3)
        R[2, 2] *= -1
        return R

    normalization = (1 - c) / (s**2)

    R = np.eye(3) + skew_cross + (skew_squared * normalization)

    # Address the reflection case
    if np.linalg.det(R) < 0:
        R[:, 3] *= -1

    return R


def skew_symmetric_cross(vector: List[float]) -> np.ndarray:
    """
    Returns the skew symmetric matrix representation of a vector.

    Args:
        a (List[float]): The vector to find the skew symmetric matrix
            representation of.

    Returns:
        np.ndarray: The skew-symmetric cross product matrix of the vector.
    """
    # [1] https://en.wikipedia.org/wiki/Cross_product#Skew-symmetric_matrix
    assert len(vector) == 3, "a must be in R3"
    skew_symm = np.array(
        [
            [+0.00, -vector[2], +vector[1]],
            [+vector[2], +0.00, -vector[0]],
            [-vector[1], +vector[0], +0.00],
        ],
        dtype=np.float32,
    )
    return skew_symm


def deskew(matrix: npt.NDArray) -> np.ndarray:
    """
    Finds the original vector from its skew-symmetric cross product. Finds the
    reverse of :meth:`mil_tools.skew_symmetric_cross`.

    Args:
        matrix (List[float]): The matrix to find the original vector from.

    Return:
        numpy.typing.NDArray: The original vector.
    """
    return np.array([matrix[2, 1], matrix[0, 2], matrix[1, 0]], dtype=np.float32)


def normalize(vector) -> npt.NDArray:
    """
    Normalizes a vector by dividing a non-zero vector by the vector norm.

    Args:
        vector (numpy.typing.ArrayLike): An array to compute the normal vector of.

    Returns:
        numpy.typing.NDArray: The normalized vector.
    """
    return vector / np.linalg.norm(vector)


def compose_transformation(
    rotation: npt.NDArray, translation: npt.NDArray
) -> np.ndarray:
    """
    Compose a transformation from a rotation matrix and a translation matrix.

    Args:
        rotation (np.ndarray): The rotation to add to the final transformation matrix.
        translation (np.ndarray): The translation to add to the final
            transformation matrix.

    Returns:
        np.ndarray: The transformation matrix.
    """
    transformation = np.zeros((4, 4))
    transformation[:3, :3] = rotation
    transformation[3, :3] = translation
    transformation[3, 3] = 1.0
    return transformation


def project_pt_to_plane(point, plane_normal):
    """
    Not currently used anywhere throughout repository.
    """
    dist = np.dot(plane_normal, point)
    projected = point - (dist * plane_normal)
    return projected


def clip_norm(vector, lower_bound, upper_bound):
    """
    Not currently used anywhere throughout repository.

    Return a vector pointing the same direction as $vector,
    with maximum norm $bound
    if norm(vector) < bound, return vector unchanged

    Like np.clip, but for vector norms
    """
    norm = np.linalg.norm(vector)
    if lower_bound < norm < upper_bound:
        return np.copy(vector)
    if norm < lower_bound:
        v_new = (vector * lower_bound) / norm
    else:
        v_new = (vector * upper_bound) / norm
    return v_new


def quaternion_matrix(q):
    mat_h = trans.quaternion_matrix(q)
    return mat_h[:3, :3] / mat_h[3, 3]


def quat_to_euler(q):
    """
    Approximate a quaternion as a euler rotation vector
    """
    euler_rot_vec = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
    final = np.array([euler_rot_vec[0], euler_rot_vec[1], euler_rot_vec[2]])
    return final


def quat_to_rotvec(quat: List[float]) -> np.ndarray:
    """
    Convert a quaternion to a rotation vector.

    Args:
        quat (List[float]): An array representing the quaternion.

    Returns:
        np.ndarray: The new rotation vector.
    """
    # For unit quaternion, return 0 0 0
    if np.all(np.isclose(quat[0:3], 0)):
        return np.array([0.0, 0.0, 0.0])
    if quat[3] < 0:
        quat = -quat
    quat = trans.unit_vector(quat)
    angle = np.arccos(quat[3]) * 2
    norm = np.linalg.norm(quat)
    axis = quat[0:3] / norm
    return axis * angle


def euler_to_quat(rotvec: List[float]) -> Quaternion:
    """
    Convert a euler rotation vector into a ROS Quaternion message.

    Args:
        rotvec (List[float]): The rotation vector to form into the Quaternion
            message.

    Returns:
        Quaternion: The newly constructed Quaternion message.
    """
    quat = trans.quaternion_from_euler(rotvec[0], rotvec[1], rotvec[2])
    return Quaternion(quat[0], quat[1], quat[2], quat[3])


def random_pose(_min, _max):
    """
    Gives a random pose in the xyz range `_min` to `_max`
    """
    pos = np.random.uniform(low=_min, high=_max, size=3)
    quat = trans.random_quaternion()
    return numpy_quat_pair_to_pose(pos, quat)
