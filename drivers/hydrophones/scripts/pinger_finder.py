#!/usr/bin/env python
import rospy
import numpy as np
from sklearn.preprocessing import normalize
import tf
from hydrophones.srv import FindPinger, SetFrequency, SetFrequencyResponse
from hydrophones.msg import ProcessedPing
from geometry_msgs.msg import Point, PoseStamped, Pose
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, SetBoolResponse
import mil_tools

def LS_Intersection3d(start, end):
    '''
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    '''
    assert len(start) == len(end)
    assert len(start) > 1
    dir_vecs = end - start
    normalize(dir_vecs, copy=False) # sklearn
    nx = dir_vecs[:, 0]
    ny = dir_vecs[:, 1]
    nz = dir_vecs[:, 2]
    XX = nx * nx - 1
    YY = ny * ny - 1
    ZZ = nz * nz - 1
    XY = nx * ny - 1
    XZ = nx * nz - 1
    YZ = ny * nz - 1
    AX = start[:, 0]
    AY = start[:, 1]
    AZ = start[:, 2]
    S = np.array([[np.sum(XX), np.sum(XY), np.sum(XZ)],
                  [np.sum(XY), np.sum(YY), np.sum(YZ)],
                  [np.sum(XZ), np.sum(YZ), np.sum(ZZ)]])
    CX = np.sum(AX * XX + AY * XY + AZ * XZ)
    CY = np.sum(AX * XY + AY * YY + AZ * YZ)
    CZ = np.sum(AX * XZ + AY * YZ + AZ * ZZ)
    C = np.stack((CX, CY, CZ))
    return np.linalg.lstsq(S, C)[0]

def LS_intersection2d(start, end):
    """
    Find the intersection of lines in the least-squares sense.
    start - Nx3 numpy array of start points
    end - Nx3 numpy array of end points
    https://en.wikipedia.org/wiki/Line-line_intersection#In_two_dimensions_2
    """
    assert len(start) == len(end)
    assert len(start) > 1
    dir_vecs = end - start
    normalize(dir_vecs, copy=False)
    Rl_90 = np.array([[0, -1], [1, 0]])  # rotates right 90deg
    perp_unit_vecs = Rl_90.dot(dir_vecs.T).T
    A_sum = np.zeros((2, 2))
    Ap_sum = np.zeros((2, 1))

    for x, y in zip(start, perp_unit_vecs):
        A = y.reshape(2, 1).dot(y.reshape(1, 2))
        Ap = A.dot(x.reshape(2, 1))
        A_sum += A
        Ap_sum += Ap

    return np.linalg.lstsq(A, Ap_sum)[0]

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
        self.map_frame = "/map"
        self.hydrophone_frame = "/hydrophones"
        self.target_freq = 35E3
        self.min_amp = 200  # if too many outliers, increase this
        self.max_observations = 200
	self.heading_start = np.empty((0, 3), float)
	self.heading_end = np.empty((0, 3), float)
        self.observation_amplitudes = np.empty((0, 0), float)
        self.pinger_position = Point(0, 0, 0)
        self.heading_pseudorange = 10
        self.debug_arrow_id = 0
        self.debug_arrow_lifetime = rospy.Duration(10, 0)
        self.listen = False
        self.ping_sub = rospy.Subscriber("/hydrophones/processed", ProcessedPing, callback=self.ping_cb)
        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
        self.pinger_finder_srv = rospy.Service('hydrophones/find_pinger', FindPinger, self.find_pinger)
        self.activate_listening = rospy.Service('hydrophones/set_listen', SetBool, self.set_listen)
        self.set_freq_srv = rospy.Service('hydrophones/set_freq', SetFrequency, self.set_freq)

    def getHydrophonePose(self, time):
        '''
        Gets the map frame pose of the hydrophone frame.
        Returns a 3-vector and a rotation matrix.
        '''
        self.tf_listener = tf.TransformListener()
        try:
            self.tf_listener.waitForTransform(
                self.map_frame, self.hydrophone_frame, time, rospy.Duration(0.25))
            trans, rot = self.tf_listener.lookupTransform(
                self.map_frame, self.hydrophone_frame, ping.header.stamp)
            rot = tf.transformations.quaternion_matrix(rot)
            return trans, rot
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print e
            return

    def ping_cb(self, ping):
        print "PINGERFINDER: freq={p.freq:.0f}  amp={p.amplitude:.0f}".format(p=ping)
        if abs(ping.amplitude) > self.min_amp and self.listen:
            p0, R = self.getHydrophonePose(ping.header.stamp)
            R = tf.transformations.quaternion_matrix(R)
            map_offset = R.dot(mil_tools.point_to_numpy(ping.position))
            p1 = p0 + map_offset
            self.visualize_arrow(p0, p1)

            self.heading_start = np.append(self.heading_start, p0, axis=0)
            self.heading_end = np.append(self.heading_end, p0, axis=0)
            self.observation_amplitudes = np.append(self.observation_amplitudes, ping.amplitude)
            if len(self.line_array) >= self.max_observations:  # delete softest samples if we have over max_observations
                softest_idx = np.argmin(self.observation_amplitudes)
                self.line_array = np.delete(self.line_array, softest_idx, axis=0)  
                self.observation_amplitudes = np.delete(self.observation_amplitudes, softest_idx)
            print "PINGERFINDER: Observation collected. Total: {}".format(len(self.heading_start))

    def find_pinger(self, srv_request):
        if len(self.heading_start) < 2:
            print "PINGER_FINDER: Can't locate pinger. Less than two valid observations have been recorded."
            return {}
        pinger_position = numpy_to_point(LS_intersection3d(self.start, self.end))
        res =  {'pinger_position' : pinger_position, 'num_samples' : len(self.heading_start)}
        self.visualize_pinger()
        return res

    def visualize_pinger_estimate(self):
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
                 

    def visualize_arrow(self, tail, head, size=None):
        if size is not None:
          head = tail + (head - tail) / np.linalg.norm(head-tail) * size
        head = Point(head[0], head[1], head[2])
        tail = Point(tail[0], tail[1], tail[2])
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
        print "PINGERFINDER: setting listening to " + ("on" if self.listen else "off")
        return {'success': True, 'message': ""}

    def set_freq(self, msg):
        self.target_freq = msg.frequency
        self.heading_start = np.empty((0, 2), float)
        self.heading_end = np.empty((0, 2), float)
        self.sample_amplitudes = np.empty((0, 0), float)
        self.pinger_position = Point(0, 0, 0)
        return SetFrequencyResponse()

if __name__ == '__main__':
    rospy.init_node('pinger_finder')
    as_per_the_ususal = PingerFinder()
    rospy.spin()
