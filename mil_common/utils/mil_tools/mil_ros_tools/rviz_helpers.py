#!/usr/bin/env python3
from typing import List, Tuple

import mil_ros_tools
import numpy as np
import rospy
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Pose, Vector3
from image_geometry import PinholeCameraModel
from std_msgs.msg import ColorRGBA

rviz_pub = rospy.Publisher("visualization", visualization_msgs.Marker, queue_size=3)


def draw_sphere(
    position: np.ndarray,
    color: np.ndarray,
    scaling: Tuple[float, float, float] = (0.11, 0.11, 0.11),
    m_id: int = 4,
    frame: str = "/base_link",
) -> None:
    """
    Draws a sphere in Rviz. Creates a new :class:`Marker` message and publishes
    it to Rviz.

    Args:
        position (np.ndarray): A list of :class:`float` that make up the point.
        color (np.ndarray): An array representing the color. The array should
            contain the ``r``, ``g``, ``b``, and ``a`` values of the color.
        scaling (Tuple[float, float, float]): The size of the sphere, in meters.
        m_id (id): The marker ID. This should be unique.
        frame (str): The frame which the sphere is relevant to.
    """
    pose = Pose(
        position=mil_ros_tools.numpy_to_point(position),
        orientation=mil_ros_tools.numpy_to_quaternion([0.0, 0.0, 0.0, 1.0]),
    )

    marker = visualization_msgs.Marker(
        ns="wamv",
        id=m_id,
        header=mil_ros_tools.make_header(frame=frame),
        type=visualization_msgs.Marker.SPHERE,
        action=visualization_msgs.Marker.ADD,
        pose=pose,
        color=ColorRGBA(*color),
        scale=Vector3(*scaling),
        lifetime=rospy.Duration(),
    )
    rviz_pub.publish(marker)


def draw_ray_3d(
    pix_coords,
    camera_model: PinholeCameraModel,
    color: ColorRGBA,
    frame: str = "/stereo_front",
    m_id=0,
    length=35,
) -> None:
    """
    Draws a 3D ray in Rviz. Creates a new Marker message and publishes it to the Rviz
    topic.

    Args:
        pix_coords (???): The coordinates of the pixel.
        camera_model (PinholeCameraModel): The camera model to derive the marker's
            direction from.
        color (ColorRGBA): The color of the ray.
        frame (str): The frame that the ray is relevant to. Defaults to ``/stereo_font``.
        m_id (int): The ID of the marker. Defaults to ``0``.
        length (int): The length of the ray. Defaults to ``35``.
    """
    marker = make_ray(
        base=np.array([0.0, 0.0, 0.0]),
        direction=np.array(camera_model.projectPixelTo3dRay(pix_coords)),
        length=length,
        color=color,
        frame=frame,
        m_id=m_id,
    )

    rviz_pub.publish(marker)


def make_ray(
    base: np.ndarray,
    direction: np.ndarray,
    length: int,
    color: List[float],
    frame: str = "/base_link",
    m_id: int = 0,
    **kwargs
) -> visualization_msgs.Marker:
    """
    Makes a ray from a base with a direction. The ray is constructed in the ``wamv``
    namespace and has an infinite lifetime.

    Args:
        base (np.ndarray): The base of the marker.
        direction (np.ndarray): The direction of the marker.
        length (int): The length of the ray.
        color (List[float]): A list containing the ``r``, ``g``, ``b``, and ``a``
            values of the color.
        frame (str): The frame that the ray is referenced from. Defaults to ``/base_link``.
        m_id (int): The unique ID of the marker. Defaults to ``0``.
    """
    marker = visualization_msgs.Marker(
        ns="wamv",
        id=m_id,
        header=mil_ros_tools.make_header(frame=frame),
        type=visualization_msgs.Marker.LINE_STRIP,
        action=visualization_msgs.Marker.ADD,
        color=ColorRGBA(*color),
        scale=Vector3(0.05, 0.05, 0.05),
        points=[Point(*o) for o in [base, direction * length]],
        lifetime=rospy.Duration(),
        **kwargs
    )
    return marker
