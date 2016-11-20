#!/usr/bin/env python
import rospy
import numpy as np
from numpy import linalg as npl
import tf
from navigator_msgs.srv import FindPinger, SetFrequency, SetFrequencyResponse
from hydrophones.msg import ProcessedPing
from geometry_msgs.msg import Point, PoseStamped, Pose
from visualization_msgs.msg import Marker
from std_srvs.srv import SetBool, SetBoolResponse
import navigator_tools

target_freq = 35000
freq_tol = 1000
min_amp = 200
max_samples = 200 # will delete entries with least amplitude after we get pass this count
line_array = np.empty((0, 4), float)
sample_amplitudes = np.empty((0, 0), float)
pinger_position = Point(0, 0, 0)
viz_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=10)
point_distance = 10  # p1 is aling the direction of the pinger but we do not know the actual
                     # range p1 will be adjusted to be 'point_distance' away from p0
debug_arrow_id = 0
listen = False

# line_arr represents the equation of a 2d line. Structure: [x1, y1, x2, y2]
# Calculates the point in the plane with the least cummulative distance to
# every line in line_arr
def LS_intersection(line_arr):
    global pinger_position
    def line_norm(line_end_pts):
        assert len(line_end_pts) == 4
        begin = line_end_pts[:2]
        end = line_end_pts[2:]
        diff = end - begin
        return npl.norm(diff)

    num_lines = line_arr.shape[0]
    begin_pts = line_arr[:, :2]
    end_pts = line_arr[:, 2:4]
    diffs = end_pts - begin_pts
    norms = np.apply_along_axis(line_norm, 1, line_arr)
    norms = norms.reshape(norms.shape[0], 1)
    perp_unit_vecs = np.apply_along_axis(lambda x: np.array([[0, -1], [1, 0]]).dot(x), 1, diffs / norms)
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
    pinger_position = Point(res[0], res[1], 0)


def find_pinger(arg):
    if line_array.shape[0] < 2:
        return None
    LS_intersection(line_array)
    return {'pinger_position' : pinger_position, 'num_samples' : line_array.shape[0]}


# receives pings and adds the representation of a line between the hydrophone array and the pinger
# to our line_arr linear equation system
def ping_cb(processed_ping):
    global line_array, sample_amplitudes
    if not listen:
        return
    print "Got processed ping message:\n{}".format(processed_ping)
    if abs(processed_ping.freq - target_freq) < freq_tol and processed_ping.amplitude > min_amp:
        print "\x1b[34mTrustworthy pinger heading\x1b[0m"
        trans, rot = None, None
        try:
            tf_listener.waitForTransform("/enu", "/hydrophones", processed_ping.header.stamp, rospy.Duration(0.25))
            trans, rot = tf_listener.lookupTransform("/enu", "/hydrophones", processed_ping.header.stamp)
            p0 = np.array([trans[0], trans[1]])
            R = tf.transformations.quaternion_matrix(rot)[:3, :3]
            print "quat: {} rot mat: {}".format(rot, R)
            print "delta hyd frame: {}".format(processed_ping.position)
            delta = R.dot(navigator_tools.point_to_numpy(processed_ping.position))[:2]
            print "delta enu frame: {}".format(delta)
            p1 = p0 + point_distance * delta / npl.norm(delta)
            line_coeffs = np.array([[p0[0], p0[1], p1[0], p1[1]]]) # p0 and p1 define a line
            visualize_line(Point(p0[0], p0[1], 0), Point(p1[0], p1[1], 0))
            line_array = np.append(line_array, line_coeffs, 0)
            sample_amplitudes = np.append(sample_amplitudes, processed_ping.amplitude)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            print e
        # delete softest samples if we have over max_samples
        if len(line_array) >= max_samples:
            softest_idx = np.argmin(sample_amplitudes)
            line_array = np.delete(line_array, softest_idx, axis=0)  
            sample_amplitudes = np.delete(sample_amplitudes, softest_idx)
        print "Number of good samples: {}".format(line_array.shape)
    else:
        print "Untrustworthy pinger heading. Freq = {} kHZ".format(processed_ping.freq)

def visualize_pinger(event):
    marker = Marker()
    marker.ns = "pinger"
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = "/enu"
    marker.type = marker.CUBE
    marker.action = marker.ADD
    marker.scale.x = 1.0
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.g = 1.0
    marker.color.a = 1.0
    marker.pose.orientation.w = 1.0
    marker.pose.position = pinger_position  
    if pinger_position != Point(0, 0, 0):
        print "POSE: ", pinger_position
        viz_pub.publish(marker)

def visualize_line(p0, p1):
    global debug_arrow_id
    marker = Marker()
    marker.ns = "pinger/debug"
    marker.header.stamp = rospy.Time(0)
    marker.header.frame_id = "/enu"
    marker.id = debug_arrow_id
    debug_arrow_id += 1
    marker.type = marker.ARROW
    marker.action = marker.ADD
    marker.lifetime = rospy.Duration(10, 0)
    marker.points.append(p0)
    marker.points.append(p1)
    marker.scale.x = 0.5
    marker.scale.y = 1.0
    marker.scale.z = 1.0
    marker.color.g = 1.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    print "viz line"
    viz_pub.publish(marker)
    
def set_listen(listen_status):
    global listen
    listen = listen_status.data
    if listen:
        print "PINGERFINDER: setting listening to on"
    else:
        print "PINGERFINDER: setting listening to off"
    return {'success': True, 'message': ""}

def set_freq(msg):
    global target_freq, line_array, sample_amplitudes, pinger_position
    target_freq = msg.frequency
    line_array = np.empty((0, 4), float)
    sample_amplitudes = np.empty((0, 0), float)
    pinger_position = Point(0, 0, 0)
    return SetFrequencyResponse()


rospy.init_node('pinger_finder')
tf_listener = tf.TransformListener()
ping_sub = rospy.Subscriber("/hydrophones/processed", ProcessedPing, callback=ping_cb)
pinger_finder_srv = rospy.Service('hydrophones/find_pinger', FindPinger, find_pinger)
activate_listening = rospy.Service('hydrophones/set_listen', SetBool, set_listen)
set_freq_srv = rospy.Service('hydrophones/set_freq', SetFrequency, set_freq)
pinger_viz = rospy.Timer(rospy.Duration(0.3), visualize_pinger)
rospy.spin()
