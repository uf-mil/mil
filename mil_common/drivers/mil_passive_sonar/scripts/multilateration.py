#!/usr/bin/python3


from collections import deque

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PointStamped, Vector3, Vector3Stamped
from mil_passive_sonar.srv import FindPinger, FindPingerResponse
from mil_ros_tools import numpy_to_point, rosmsg_to_numpy
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from std_srvs.srv import SetBool, SetBoolResponse, Trigger, TriggerResponse
from tf2_geometry_msgs import PointStamped, Vector3Stamped


class MultilaterationNode:
    """
    Node which observes the heading to the pinger from various observation points
    and predicts the absolute position from this data.
    """

    def __init__(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        buffer_len = rospy.get_param("~buffer_size", default=15)
        self.minimum_delta_dist = rospy.get_param("~min_delta", default=0.2)
        self.filter_dist = rospy.get_param("~filter_dist", default=5)
        self.filter_count = rospy.get_param("~filter_count", default=4)
        self.buffer_min = rospy.get_param("~buffer_min", default=10)
        self.vec_samples = deque(maxlen=buffer_len)
        self.enabled = False
        self.last_pos = None
        self.global_frame = rospy.get_param("~global_frame", default="enu")
        self.heading_sub = rospy.Subscriber(
            "/hydrophones/direction", Vector3Stamped, self.heading_cb, queue_size=10
        )
        self.position_pub = rospy.Publisher(
            "/hydrophones/position", PointStamped, queue_size=1
        )
        self.enable_srv = rospy.Service("~enable", SetBool, self.enable_cb)
        self.reset_srv = rospy.Service("~reset", Trigger, self.reset_cb)

    def enable_cb(self, req):
        self.enabled = req.data
        return SetBoolResponse(success=True)

    def reset_cb(self, req):
        self.vec_samples.clear()
        return {"success": True}

    def heading_cb(self, p_message):
        if not self.enabled:
            return
        try:
            # Transform ping into global frame
            transformed_vec = self.tfBuffer.transform(
                p_message, self.global_frame, rospy.Duration(2)
            )
            transformed_origin = self.tfBuffer.lookup_transform(
                self.global_frame,
                p_message.header.frame_id,
                p_message.header.stamp,
                rospy.Duration(2),
            )
            vec = rosmsg_to_numpy(transformed_vec.vector)
            vec = vec / np.linalg.norm(vec)
            origin = rosmsg_to_numpy(transformed_origin.transform.translation)
        except tf2_ros.TransformException as e:
            rospy.logwarn(f"TF Exception: {e}")
            return

        # Check if two samples were taken too close to each other
        if (
            self.last_pos is not None
            and np.linalg.norm(self.last_pos - origin) < self.minimum_delta_dist
        ):
            rospy.loginfo("Observation too close to previous, ignoring")
            return
        self.last_pos = origin

        # Add ping to samples list
        self.vec_samples.append((origin, origin + vec))

        # If there are enough samples, calculate position
        if len(self.vec_samples) >= self.buffer_min:
            self.publish_position()

    def publish_position(self):
        # Setup lines from samples
        line_array = np.array(
            [(np.array(p[0][0:2]), np.array(p[1][0:2])) for p in self.vec_samples]
        )

        # Calculate least squares intersection
        where = self.ls_line_intersection2d(line_array)

        # If no point could be determined, don't publish
        if where is None:
            rospy.logwarn_throttle(
                10.0,
                "Could not determine point; likely because too many lobs were filtered out due to intersections close to their origins",
            )
            return

        # Publish point
        ps = PointStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = rospy.Time.now()
        ps.point = numpy_to_point(where)
        self.position_pub.publish(ps)

    def ls_line_intersection2d(self, lines_array):
        """
        Find the intersection of lines in the least-squares sense.
        Filter lobs such that no lob can have more than a number of intersections with other lines within a radius of the boat
        """

        # Filter out lobs that intersect with many other lobs close to their origin

        line_array = np.array([[0, 0, 0, 0]])
        line_intersecting_array = np.array([])

        for line1 in lines_array:
            line_intersections = 0
            for line2 in lines_array:
                if np.array_equal(line1, line2):
                    continue

                boat_pos1 = line1[0]
                boat_pos2 = line2[0]

                # Calculate the intersection of the two lines
                # https://stackoverflow.com/a/47823499
                t, s = np.linalg.solve(
                    np.array([line1[1] - line1[0], line2[0] - line2[1]]).T,
                    line2[0] - line1[0],
                )
                intersection = (1 - t) * line1[0] + t * line1[1]

                # Calculate the distance to the boat's position from the intersection
                distance1 = np.linalg.norm(intersection - boat_pos1)
                distance2 = np.linalg.norm(intersection - boat_pos2)

                # If the distances are within the filter radius, increment the intersections count
                if distance1 < self.filter_dist or distance2 < self.filter_dist:
                    line_intersections += 1

            if line_intersections < self.filter_count:
                line_array = np.vstack(
                    [
                        line_array,
                        np.array([line1[0][0], line1[0][1], line1[1][0], line1[1][1]]),
                    ]
                )

        line_array = line_array[1:]

        # If we don't have at least 3 lines left, return no results
        if len(line_array) < 3:
            return None

        # https://en.wikipedia.org/wiki/Line-line_intersection#In_two_dimensions_2

        def line_segment_norm(line_end_pts):
            assert len(line_end_pts) == 4
            return np.linalg.norm(line_end_pts[2:] - line_end_pts[:2])

        begin_pts = line_array[:, :2]
        diffs = line_array[:, 2:4] - begin_pts
        norms = np.apply_along_axis(line_segment_norm, 1, line_array).reshape(
            diffs.shape[0], 1
        )
        rot_left_90 = np.array([[0, -1], [1, 0]])
        perp_unit_vecs = np.apply_along_axis(
            lambda unit_diffs: rot_left_90.dot(unit_diffs), 1, diffs / norms
        )
        A_sum = np.zeros((2, 2))
        Ap_sum = np.zeros((2, 1))

        for x, y in zip(begin_pts, perp_unit_vecs):
            begin = x.reshape(2, 1)
            perp_vec = y.reshape(2, 1)
            A = perp_vec.dot(perp_vec.T)
            Ap = A.dot(begin)
            A_sum += A
            Ap_sum += Ap

        return np.linalg.inv(A_sum).dot(Ap_sum)


if __name__ == "__main__":
    rospy.init_node("multilateration")
    MultilaterationNode()
    rospy.spin()
