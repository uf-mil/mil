#!/usr/bin/env python
import txros
import numpy as np
from vrx import Vrx
from twisted.internet import defer
from sensor_msgs.msg import PointCloud2
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_vision_tools.cv_tools import roi_enclosing_points, rect_from_roi
from image_geometry import PinholeCameraModel
from sensor_msgs.msg import Image, CameraInfo
from operator import attrgetter
from std_srvs.srv import SetBoolRequest
from navigator_vision import VrxColorClassifier

LED_PANNEL_MAX  = 0.25
LED_PANNEL_MIN = 0.6

STC_HEIGHT = 2.3
STC_WIDTH = 2

CAMERA_TOPIC = '/wamv/sensors/cameras/front_left_camera/image_raw'
CAMERA_INFO_TOPIC = '/wamv/sensors/cameras/front_left_camera/camera_info'

class ScanTheCode(Vrx):

    def __init__(self):
        super(ScanTheCode, self).__init__()
        self.camera_model = PinholeCameraModel()
        self.classifier = VrxColorClassifier()
        self.classifier.train_from_csv()

    @classmethod
    def init(cls):
        cls.debug_points_pub = cls.nh.advertise('/stc_led_points', PointCloud2)
        cls.camera_sub = cls.nh.subscribe(CAMERA_TOPIC, Image)
        cls.camera_info_sub = cls.nh.subscribe(CAMERA_INFO_TOPIC, CameraInfo)

    @txros.util.cancellableInlineCallbacks
    def run(self, args):

        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))
        # see if we got scan the code tower
        try:
            _, pose = yield self.get_sorted_objects(name='stc_platform', n=1)
            pose = pose[0]
        except Exception as e:
            try:
                _, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
            except Exception as e:
                yield self.move.forward(20).go()
                _, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
            # go to avg of those objs to get better data on those points
            print 'going to center of things'
            yield self.move.set_position(sum(poses)/len(poses)).go()
            msgs, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
            most_stc = msgs[0]
            pose_idx = 0
            least_error = self.stc_error(most_stc.scale)
            for i, msg in enumerate(msgs):
                if self.stc_error(msg.scale) < least_error:
                    least_error = self.stc_error(msg.scale)
                    most_stc = msg
                    pose_idx = i
            cmd = '%d=stc_platform'%pose_idx
            yield self.get_sorted_objects(name='', n=-1, throw=True, cmd=cmd)
            _, pose = yield self.get_sorted_objects(name='stc_platform', n=1)
            pose = pose[0]
        yield self.move.look_at(pose).go()
        yield self.move.set_position(pose).backward(5).go()

        # get updated points tf img and info now that we a closer

        select = defer.DeferredList([
            self.get_sorted_objects(name='stc_platform', n=1),
            self.tf_listener.get_transform('front_left_camera_link_optical', 'enu'),
            self.camera_sub.get_next_message(),
            self.camera_info_sub.get_next_message()])

        #stc, tf, img, info = yield result.resultList
        result_list = []
        while select.finishedCount != 4:
            a = yield select
            print a
            result_list[idx] = result
        #print dir(result.resultList[0])
        stc, tf, img, info = result_list
        print dir(stc)# = stc[0]
        # do a z filter for the led points
        top = max(stc.points, key=attrgetter('z')).z
        points = np.array([[i.x, i.y, i.z] for i in stc.points 
                            if i.z < top-LED_PANNEL_MAX and i.z > top-LED_PANNEL_MIN])
        msg = np2pc2(points, self.nh.get_time(), 'enu')
        self.debug_points_pub.publish(msg)
        yield self.camera_model.fromCameraInfo(info)
        for i in range(len(points)):
            points[i] = tf.transform_point(points[i])
        roi = roi_enclosing_points(self.camera_model, points)
        rect = rect_from_roi(roi)
        bbox_contour = bbox_countour_from_rectangle(rect)
        


        self.send_feedback('Done!')

    def stc_error(self, a):
        '''
        a: geometry_msg/Vector3. scale of potential stc pcodar object
        returns: the squared error of a bounding box around an object from the stc box
        '''
        return np.linalg.norm(np.array([(a.z - STC_HEIGHT)**2,
                                        (a.x - STC_WIDTH)**2,
                                        (a.y - STC_WIDTH)**2]))
