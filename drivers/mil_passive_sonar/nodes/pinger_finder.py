#!/usr/bin/env python
import rospy
import rospkg
import numpy as np
from numpy import linalg as npl
import tf
from navigator_msgs.srv import FindPinger, SetFrequency, SetFrequencyResponse
from hydrophones.msg import ProcessedPing
from geometry_msgs.msg import Point, PoseStamped, Pose
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, SetBoolResponse
import navigator_tools


class PingerFinder(object):
    """
    This class will find the position of a pinger at a given frequency. For it 
    to find a good solution, it needs to get observations from at least 2 sufficiently
    different positions. Observations are considered valid if they fall within 
    self.freq_tol of self.target_freq, where self.target_freq can be set
    using 'rosservice call /hydrophones/set_freq "frequency: ____"'.

    The default behaviour is to not store observations. To start recording valid 
    observations, run 'rosservice call /hydrophones/set_listen "data: true"'. Remember
    to run 'rosservice call /hydrophones/set_listen "data: false"' to stop recording
    observations. For best results, it is best to only record observations while the
    vehicle is station-keeping.

    When running, this class will publish arrow markers to /visualization_marker and
    a green cube marker representing the estimated position of the pinger whenever the
    /hydrophones/find_pinger service is called.
    """

    def __init__(self):
        self.map_frame = "/enu"
        self.hydrophone_frame = "/hydrophones"
        self.tf_listener = tf.TransformListener()
        self.target_freq = 35E3
        self.freq_tol = 1E3
        self.min_amp = 200  # if too many outliers, increase this
        self.max_observations = 200
        self.line_array = np.empty((0, 4), float)
        self.observation_amplitudes = np.empty((0, 0), float)
        self.pinger_position = Point(0, 0, 0)
        self.heading_pseudorange = 10
        self.debug_arrow_id = 0
        self.debug_arrow_lifetime = rospy.Duration(10, 0)
        self.listen = False
        self.ping_sub = rospy.Subscriber(
            "/hydrophones/processed", ProcessedPing, callback=self.ping_cb)
        self.marker_pub = rospy.Publisher(
            "/visualization_marker", Marker, queue_size=10)
        self.pinger_finder_srv = rospy.Service(
            'hydrophones/find_pinger', FindPinger, self.find_pinger)
        self.activate_listening = rospy.Service(
            'hydrophones/set_listen', SetBool, self.set_listen)
        self.set_freq_srv = rospy.Service(
            'hydrophones/set_freq', SetFrequency, self.set_freq)

    def ping_cb(self, ping):
        print rospkg.get_ros_package_path()
        print "PINGERFINDER: freq={p.freq:.0f}  amp={p.amplitude:.0f}".format(p=ping)
        if abs(ping.freq - self.target_freq) < self.freq_tol and ping.amplitude > self.min_amp and self.listen:
            trans, rot = None, None
            try:
                self.tf_listener.waitForTransform(
                    self.map_frame, self.hydrophone_frame, ping.header.stamp, rospy.Duration(0.25))
                trans, rot = self.tf_listener.lookupTransform(
                    self.map_frame, self.hydrophone_frame, ping.header.stamp)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                print e
                return
            p0 = np.array([trans[0], trans[1]])
            R = tf.transformations.quaternion_matrix(rot)[:3, :3]
            delta = R.dot(navigator_tools.point_to_numpy(ping.position))[:2]
            p1 = p0 + self.heading_pseudorange * delta / npl.norm(delta)
            # p0 and p1 define a line
            line_coeffs = np.array([[p0[0], p0[1], p1[0], p1[1]]])
            self.visualize_arrow(
                Point(p0[0], p0[1], 0), Point(p1[0], p1[1], 0))
            self.line_array = np.append(self.line_array, line_coeffs, 0)
            self.observation_amplitudes = np.append(
                self.observation_amplitudes, ping.amplitude)
            # delete softest samples if we have over max_observations
            if len(self.line_array) >= self.max_observations:
                softest_idx = np.argmin(self.observation_amplitudes)
                self.line_array = np.delete(
                    self.line_array, softest_idx, axis=0)
                self.observation_amplitudes = np.delete(
                    self.observation_amplitudes, softest_idx)
            print "PINGERFINDER: Observation collected. Total: {}".format(self.line_array.shape[0])

    def LS_intersection(self):
        """
        self.line_array represents a system of 2d line equations. Each row represents a different
        observation of a line in map frame on which the pinger lies. Row structure: [x1, y1, x2, y2]
        Calculates the point in the plane with the least cummulative distance to every line
        in self.line_array. For more information, see:
        https://en.wikipedia.org/wiki/Line-line_intersection#In_two_dimensions_2
        """

        def line_segment_norm(line_end_pts):
            assert len(line_end_pts) == 4
            return npl.norm(line_end_pts[2:] - line_end_pts[:2])

        begin_pts = self.line_array[:, :2]
        diffs = self.line_array[:, 2:4] - begin_pts
        norms = np.apply_along_axis(
            line_segment_norm, 1, self.line_array).reshape(diffs.shape[0], 1)
        rot_left_90 = np.array([[0, -1], [1, 0]])
        perp_unit_vecs = np.apply_along_axis(
            lambda unit_diffs: rot_left_90.dot(unit_diffs), 1, diffs / norms)
        A_sum = np.zeros((2, 2))
        Ap_sum = np.zeros((2, 1))

        for x, y in zip(begin_pts, perp_unit_vecs):
            begin = x.reshape(2, 1)
            perp_vec = y.reshape(2, 1)
            A = perp_vec.dot(perp_vec.T)
            Ap = A.dot(begin)
            A_sum += A
            Ap_sum += Ap

        res = npl.inv(A_sum).dot(Ap_sum)
        self.pinger_position = Point(res[0], res[1], 0)
        return self.pinger_position

    def find_pinger(self, srv_request):
        if self.line_array.shape[0] < 2:
            print "PINGER_FINDER: Can't locate pinger. Less than two valid observations have been recorded."
            return {}
        res = {'pinger_position': self.LS_intersection(
        ), 'num_samples': self.line_array.shape[0]}
        self.visualize_pinger()
        return res

    def visualize_pinger(self):
        marker = Marker()
        marker.ns = "pinger{}".format(self.target_freq)
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = self.map_frame
        marker.type = marker.CUBE
        marker.action = marker.ADD
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.g = 1.0
        marker.color.a = 0.75
        marker.pose.position = self.pinger_position
        if self.pinger_position != Point(0, 0, 0):
            self.marker_pub.publish(marker)
            print "PINGERFINDER: position: ({p.x[0]:.2f}, {p.y[0]:.2f})".format(p=self.pinger_position)

    def visualize_arrow(self, tail, head):
        marker = Marker()
        marker.ns = "pinger{}/heading".format(self.target_freq)
        marker.header.stamp = rospy.Time(0)
        marker.header.frame_id = self.map_frame
        marker.id = self.debug_arrow_id
        self.debug_arrow_id += 1
        marker.type = marker.ARROW
        marker.action = marker.ADD
        marker.lifetime = self.debug_arrow_lifetime
        marker.points.append(tail)
        marker.points.append(head)
        marker.scale.x = 0.5
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.5
        self.marker_pub.publish(marker)

    def set_listen(self, listen_status):
        self.listen = listen_status.data
        print "PINGERFINDER: setting listening to on" if self.listen else "PINGERFINDER: setting listening to off"
        return {'success': True, 'message': ""}

    def set_freq(self, msg):
        self.target_freq = msg.frequency
        self.line_array = np.empty((0, 4), float)
        self.sample_amplitudes = np.empty((0, 0), float)
        self.pinger_position = Point(0, 0, 0)
        return SetFrequencyResponse()


if __name__ == '__main__':
    rospy.init_node('pinger_finder')
    as_per_the_ususal = PingerFinder()
    rospy.spin()
