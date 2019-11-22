#!/usr/bin/env python
import txros
import numpy as np
from vrx import Vrx
from twisted.internet import defer
from sensor_msgs.msg import PointCloud2
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_vision_tools.cv_tools import rect_from_roi, roi_enclosing_points, contour_mask
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from operator import attrgetter
from std_srvs.srv import SetBoolRequest
from navigator_vision import VrxStcColorClassifier
from cv_bridge import CvBridge
from cv2 import bitwise_and
from vrx_gazebo.srv import ColorSequenceRequest, ColorSequence
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from dynamic_reconfigure.msg import DoubleParameter

LED_PANNEL_MAX  = 0.25
LED_PANNEL_MIN = 0.6

STC_HEIGHT = 2.3
STC_WIDTH = 2

CAMERA_TOPIC = '/wamv/sensors/cameras/front_left_camera/image_raw'
CAMERA_INFO_TOPIC = '/wamv/sensors/cameras/front_left_camera/camera_info'
CAMERA_LINK_OPTICAL = 'front_left_camera_link_optical'

COLOR_SEQUENCE_SERVICE = '/vrx/scan_dock/color_sequence'

TIMEOUT_SECONDS = 30

class ScanTheCode(Vrx):

    def __init__(self, *args, **kwargs):
        super(ScanTheCode, self).__init__(*args, **kwargs)
        self.classifier = VrxStcColorClassifier()
        self.classifier.train_from_csv()
        self.camera_model = PinholeCameraModel()

    @classmethod
    def init(cls):
        cls.debug_points_pub = cls.nh.advertise('/stc_led_points', PointCloud2)
        cls.camera_sub = cls.nh.subscribe(CAMERA_TOPIC, Image)
        cls.camera_info_sub = cls.nh.subscribe(CAMERA_INFO_TOPIC, CameraInfo)
        cls.bridge = CvBridge()
        cls.image_debug_pub = cls.nh.advertise('/stc_mask_debug', Image)
        cls.sequence_report = cls.nh.get_service_client(COLOR_SEQUENCE_SERVICE, ColorSequence)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):

        info = yield self.camera_info_sub.get_next_message()
        self.camera_model.fromCameraInfo(info)

        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))

        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = 'cluster_tolerance_m'
        pcodar_cluster_tol.value = 20

        yield self.pcodar_set_params(doubles = [pcodar_cluster_tol])
        yield self.find_stc()
        _, pose = yield self.get_sorted_objects(name='stc_platform', n=1)
        pose = pose[0]
        yield self.move.look_at(pose).set_position(pose).backward(5).go()
        # get updated points and tf now that we a closer

        stc_query = yield self.get_sorted_objects(name='stc_platform', n=1)
        stc = stc_query[0][0]
        tf = yield self.tf_listener.get_transform(CAMERA_LINK_OPTICAL, 'enu')
        points = z_filter(stc)

        msg = np2pc2(points, self.nh.get_time(), 'enu')
        self.debug_points_pub.publish(msg)

        points = np.array([tf.transform_point(points[i]) for i in range(len(points))])

        contour = np.array(bbox_from_rect(
            rect_from_roi(roi_enclosing_points(self.camera_model, points))), dtype=int)
        try:
            sequence = yield txros.util.wrap_timeout(self.get_sequence(contour), TIMEOUT_SECONDS, 'Guessing RGB')
        except txros.util.TimeoutError:
            sequence = ['red', 'green', 'blue']
        print 'Scan The Code Color Sequence', sequence
        self.report_sequence(sequence)
        yield self.send_feedback('Done!')
        defer.returnValue(sequence)

    @txros.util.cancellableInlineCallbacks
    def get_sequence(self, contour):
        sequence = []
        while len(sequence) < 3:
            img = yield self.camera_sub.get_next_message()
            img = self.bridge.imgmsg_to_cv2(img)

            mask = contour_mask(contour, img_shape=img.shape)

            img = img[:,:,[2,1,0]]
            mask_msg = self.bridge.cv2_to_imgmsg(bitwise_and(img, img, mask = mask))

            self.image_debug_pub.publish(mask_msg)
            features = np.array(self.classifier.get_features(img, mask)).reshape(1, 9)
            #print features
            class_probabilities = self.classifier.feature_probabilities(features)[0]
            most_likely_index = np.argmax(class_probabilities)
            most_likely_name = self.classifier.CLASSES[most_likely_index]
            probability = class_probabilities[most_likely_index]
            #print most_likely_name
            if most_likely_name == 'off':
                sequence = []
            elif sequence == [] or  most_likely_name != sequence[-1]:
                sequence.append(most_likely_name)
        defer.returnValue(sequence)

    @txros.util.cancellableInlineCallbacks
    def report_sequence(self, sequence):
        color_sequence = ColorSequenceRequest()
        color_sequence.color1 = sequence[0]
        color_sequence.color2 = sequence[1]
        color_sequence.color3 = sequence[2]

        try:
            yield self.sequence_report(color_sequence)
        except Exception as e: #catch error incase vrx scroing isnt running
            print e

    @txros.util.cancellableInlineCallbacks
    def label_stc(self):
        msgs, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
        most_stc = msgs[0]
        pose_idx = 0
        least_error = stc_error(most_stc.scale)
        for msg in msgs:
            if stc_error(msg.scale) < least_error:
                least_error = stc_error(msg.scale)
                most_stc = msg
        cmd = '%d=stc_platform'%most_stc.id
        yield self._database_query(ObjectDBQueryRequest(name='', cmd=cmd))


    @txros.util.cancellableInlineCallbacks
    def find_stc(self):
        # see if we already got scan the code tower
        try:
            _, pose = yield self.get_sorted_objects(name='stc_platform', n=1)
            pose = pose[0]
        # incase stc platform not already identified
        except Exception as e:
            # get all pcodar objects
            try:
                _, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
            # if no pcodar objects, drive forward
            # TODO: replace with actual explore
            except Exception as e:
                yield self.move.forward(50).go()
                # get all pcodar objects
                try:
                    _, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
                # if still no pcodar objects, guess RGB and exit mission
                except Exception as e:
                    sequence = ['red', 'green', 'blue']
                    yield self.report_sequence(sequence)
                    defer.returnValue(sequence)
            # go to avg of those objs to get better data on those objs
            print 'going to center of things'
            yield self.move.set_position(sum(poses)/len(poses)).go()
            yield self.label_stc()

def z_filter(db_obj_msg):
    # do a z filter for the led points
    top = max(db_obj_msg.points, key=attrgetter('z')).z
    points = np.array([[i.x, i.y, i.z] for i in db_obj_msg.points 
                        if i.z < top-LED_PANNEL_MAX and i.z > top-LED_PANNEL_MIN])
    return points

def stc_error(a):
    '''
    a: geometry_msg/Vector3. scale of potential stc pcodar object
    returns: the squared error of a bounding box around an object from the stc box
    '''
    return np.linalg.norm(np.array([(a.z - STC_HEIGHT)**2,
                                    (a.x - STC_WIDTH)**2,
                                    (a.y - STC_WIDTH)**2]))

def bbox_from_rect(rect):
    bbox = np.array([[rect[0][0], rect[0][1]],
                     [rect[1][0], rect[0][1]],
                     [rect[1][0], rect[1][1]],
                     [rect[0][0], rect[1][1]]])
    return bbox
