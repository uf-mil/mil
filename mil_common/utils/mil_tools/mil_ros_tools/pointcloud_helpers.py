from typing import Optional

import numpy as np
from genpy.rostime import Time
from sensor_msgs.msg import PointCloud2, PointField


def numpy_to_pointcloud2(
    points: np.ndarray,
    stamp: Optional[Time] = None,
    frame_id: Optional[str] = None,
) -> PointCloud2:
    """
    Create a sensor_msgs.PointCloud2 from an array of points.

    Args:
        points (np.ndarray): The array of points.
        stamp (Optional[genpy.rostime.Time]): An optional timestamp for the message header.
        frame_id (Optional[str]): An optional string describing the frame
          associated with the point cloud.

    Returns:
        PointCloud2: A PointCloud2 message with the provided information.
    """
    msg = PointCloud2()
    if stamp:
        msg.header.stamp = stamp
    if frame_id:
        msg.header.frame_id = frame_id
    if len(points.shape) == 3:
        msg.height = points.shape[1]
        msg.width = points.shape[0]
    else:
        msg.height = 1
        msg.width = len(points)
    msg.fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * points.shape[0]
    msg.is_dense = int(np.isfinite(points).all())
    msg.data = np.asarray(points, np.float32).tostring()

    return msg
