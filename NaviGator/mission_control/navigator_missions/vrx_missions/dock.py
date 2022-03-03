#!/usr/bin/env python
import txros
import numpy as np
from vrx import Vrx
from twisted.internet import defer
from sensor_msgs.msg import PointCloud2
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_vision_tools.cv_tools import rect_from_roi, roi_enclosing_points, contour_mask
from image_geometry import PinholeCameraModel
from operator import attrgetter
from std_msgs.msg import Empty
from std_srvs.srv import SetBoolRequest
from navigator_vision import VrxStcColorClassifier
from cv_bridge import CvBridge
import cv2
import os
from tf import transformations
from rospkg import RosPack
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from dynamic_reconfigure.msg import DoubleParameter
from mil_tools import pose_to_numpy, rosmsg_to_numpy
from tf.transformations import quaternion_matrix
from sensor_msgs.msg import Image, CameraInfo

PANNEL_MAX  = 0
PANNEL_MIN = 2

CAMERA_LINK_OPTICAL = 'wamv/front_left_camera_link_optical'

COLOR_SEQUENCE_SERVICE = '/vrx/scan_dock/color_sequence'

TIMEOUT_SECONDS = 30

class Dock(Vrx):

    def __init__(self, *args, **kwargs):
        super(Dock, self).__init__(*args, **kwargs)
        self.classifier = VrxStcColorClassifier()
        self.classifier.train_from_csv()
        self.camera_model = PinholeCameraModel()
        self.rospack = RosPack()

    @txros.util.cancellableInlineCallbacks
    def run(self, args):
        self.debug_points_pub = self.nh.advertise('/dock_pannel_points', PointCloud2)
        self.bridge = CvBridge()

        self.image_debug_pub = self.nh.advertise('/dock_mask_debug', Image)
        self.init_front_left_camera()
        args = str.split(args, ' ')
        self.color = args[0]
        self.shape = args[1]

        print("entered docking task", self.color, self.shape)

        yield self.set_vrx_classifier_enabled(SetBoolRequest(data=False))
        info = yield self.front_left_camera_info_sub.get_next_message()
        self.camera_model.fromCameraInfo(info)

        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = 'cluster_tolerance_m'
        pcodar_cluster_tol.value = 10

        yield self.pcodar_set_params(doubles = [pcodar_cluster_tol])
        self.nh.sleep(5)

        pos = yield self.find_dock()

        print("going towards dock")
        yield self.move.look_at(pos).set_position(pos).backward(20).go()

        # get a vector to the longer side of the dock
        dock, pos = yield self.get_sorted_objects(name='dock', n=1)
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        bbox = rosmsg_to_numpy(dock.scale)
        bbox[2] = 0
        max_dim = np.argmax(bbox[:2])
        bbox[max_dim] = 0
        bbox_enu = np.dot(rotation[:3,:3],bbox)
        #this black magic uses the property that a rotation matrix is just a 
        #rotated cartesian frame and only gets the vector that points towards
        #the longest side since the vector pointing that way will be at the 
        #same index as the scale for the smaller side. This is genius!
        # - Andrew Knee

        #move to first attempt
        print("moving in front of dock")
        goal_pos = None
        curr_pose = yield self.tx_pose
        side_a_bool = False
        side_b_bool = False
        side_a = bbox_enu+position
        side_b = -bbox_enu+position

        if np.linalg.norm(side_a - curr_pose[0]) < np.linalg.norm(side_b - curr_pose[0]):
            goal_pos = side_a
            side_a_bool = True
        else:
            goal_pos = side_b
            side_b_bool = True

        yield self.move.set_position(goal_pos).look_at(position).go()

        target_symbol = self.color + "_" + self.shape
        symbol_position = yield self.get_symbol_position(target_symbol)
        print("The correct docking location is ", symbol_position)

        ###
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

        side_vect = 0.85 * bbox_enu
        if side_a_bool:
            left_position = np.dot(rot, side_vect) + side_a
            right_position = np.dot(rot, -side_vect) + side_a
        else:
            left_position = np.dot(rot, side_vect) + side_b
            right_position = np.dot(rot, -side_vect) + side_b

        dock_point_left = np.dot(rot, side_vect) + position
        dock_point_right = np.dot(rot, -side_vect) + position
        ###        

        #position boat in front of correct symbol
        if symbol_position == "left":
            yield self.move.set_position(left_position).look_at(dock_point_left).go(move_type="skid")
            position = dock_point_left
        elif symbol_position == "right":
            yield self.move.set_position(right_position).look_at(dock_point_right).go(move_type="skid")
            position = dock_point_right

        #enter dock
        yield self.move.forward(7).go(blind=True, move_type="skid")

        #fire ball
        self.fire_ball.publish(Empty())
        yield self.nh.sleep(2)
        self.fire_ball.publish(Empty())
        yield self.nh.sleep(2)
        self.fire_ball.publish(Empty())
        yield self.nh.sleep(2)
        self.fire_ball.publish(Empty())
        yield self.nh.sleep(4)

        #Exit dock
        yield self.move.backward(7).go(blind=True, move_type="skid")


        #for i in range(3):
        #    if curr_color!=self.color:
        #        # check next dock station
        #        yield self.move.backward(5).go()
        #        yield self.move.set_position((-bbox_enu+position)).look_at(position).go()
        #        continue
        #    else:
        #        if self.get_shape(masked_image) == self.shape:
        #            print('shapes match')
        #            yield self.dock()
        #        else:
        #            # go to other one
        #            yield self.move.backward(5).go()
        #            yield self.move.set_position((-bbox_enu+position)).look_at(position).go()
        #            curr_color, _ = yield self.get_color()
#
        #            task_info = yield self.task_info_sub.get_next_message()
        #            if curr_color!=self.color and task_info.remaining_time.secs >= 60:
        #                # if we got the right color on the other side and the wrong color on this side, 
        #                # go dock in the other side
        #                yield self.move.backward(5).go()
        #                yield self.move.set_position((bbox_enu+position)).look_at(position).go()
        #                yield self.dock()
        #            else:
        #                # this side is the right color
        #                # Dock at the other side of the dock, no further perception
        #                yield self.dock()
        yield self.send_feedback('Done!')


    @txros.util.cancellableInlineCallbacks
    def get_symbol_position(self, target_symbol):

        #voting system for ten pictures [left, center, right]
        vote = [0,0,0]
        method = eval('cv2.TM_CCOEFF_NORMED')
        path = self.rospack.get_path('navigator_vision')
        symbol_file = os.path.join(path, 'datasets/dock_target_images/' + target_symbol + ".png")
        symbol = cv2.imread(symbol_file)
        _,w,h = symbol.shape[::-1]

        #loop through ten pictures
        for i in range(10):

            img = yield self.front_left_camera_sub.get_next_message()

            img = self.bridge.imgmsg_to_cv2(img)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            _,width,_ = img.shape[::-1]

            res = cv2.matchTemplate(img,symbol,method)
            _, _, min_loc, max_loc = cv2.minMaxLoc(res)

            top_left = max_loc
            bottom_right = (top_left[0] + w, top_left[1] + h)
            cv2.rectangle(img,top_left, bottom_right, 255, 2)
            masked_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
            self.image_debug_pub.publish(masked_msg)

            if min_loc[0] < width/3.0:
                vote[0] = vote[0] + 1
            elif width/3.0 < min_loc[0] < 2*width/3.0:
                vote[1] = vote[1] + 1
            else:
                vote[2] = vote[2] + 1 

        most_likely_index = np.argmax(vote)

        symbol_position = "left"
        if most_likely_index == 0:
            symbol_position = "left"
        elif most_likely_index == 1:
            symbol_position = "center"
        elif most_likely_index == 2:
            symbol_position = "right"

        defer.returnValue(symbol_position)

    @txros.util.cancellableInlineCallbacks
    def dock(self):
        avg_point = yield self.get_placard_point()
        yield self.move.set_position(avg_point).backward(5).look_at(avg_point).go(blind=True)
        yield self.nh.sleep(11)
        for i in range(15):
            yield self.move.backward(1).look_at(avg_point).go(blind=True)

    def get_shape(self, masked_img):
        gray = cv2.cvtColor(masked_img, cv2.COLOR_RGB2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 17, 17)
        edged = cv2.Canny(gray, 30, 200)

        _, contours, _ = cv2.findContours(edged, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        masked_img = np.zeros(masked_img.shape)

        cv2.drawContours(masked_img, contours, -1, 255, 3)

        masked_msg = self.bridge.cv2_to_imgmsg(masked_img)
        self.image_debug_pub.publish(masked_msg)

        approx_area = []
        for contour in contours:
            approx = cv2.approxPolyDP(contour,0.01*cv2.arcLength(contour,True),True)
            area = cv2.contourArea(contour)
            print len(approx), area
            approx_area.append((approx,area))

        approx_area.sort(key=lambda tup: tup[1],reverse = True)
        for i in approx_area:
            if (len(i[0]) == 12):
                print('detected as cruciform')
                return 'cruciform'
            if (len(i[0]) == 3):
                print('detected as triangle')
                return('triangle')
        print('deteched as circle')
        return 'circle'



    @txros.util.cancellableInlineCallbacks
    def find_dock(self):
        # see if we already got scan the code tower
        try:
            _, poses = yield self.get_sorted_objects(name='dock', n=1)
            pose = poses[0]
        # incase stc platform not already identified
        except Exception as e:
            # get all pcodar objects
            msgs = None
            while msgs is None:
                try:
                    msgs, poses = yield self.get_sorted_objects(name='UNKNOWN', n=-1)
                except Exception as e:
                    yield self.move.forward(10).go()
            yield self.pcodar_label(msgs[0].id, 'dock')
            # if no pcodar objects, throw error, exit mission
            pose = poses[0]
        defer.returnValue(pose)

    @txros.util.cancellableInlineCallbacks
    def get_placard_point(self):
        print("entered get_placard_point")
        dock_query = yield self.get_sorted_objects(name='dock', n=1)
        print("got dock query")
        dock = dock_query[0][0]
        tf = yield self.tf_listener.get_transform(CAMERA_LINK_OPTICAL, 'enu')
        points = z_filter(dock)
        points = np.array([points[i] for i in range(len(points))])
        avg_point = sum(points)/len(points)
        defer.returnValue(avg_point)

    @txros.util.cancellableInlineCallbacks
    def get_placard_points(self):
        dock_query = yield self.get_sorted_objects(name='dock', n=1)
        dock = dock_query[0][0]
        tf = yield self.tf_listener.get_transform(CAMERA_LINK_OPTICAL, 'enu')
        points = z_filter(dock)
        points = np.array([tf.transform_point(points[i]) for i in range(len(points))])
        defer.returnValue(points)

    @txros.util.cancellableInlineCallbacks
    def get_color(self):
        print("entered get color")
        avg_point = yield self.get_placard_point()
        print("placard point")
        yield self.move.set_position(avg_point).backward(11.5).look_at(avg_point).go()
        print("moving vehicle")
        points = yield self.get_placard_points()
        print("placard points")
        msg = np2pc2(points, self.nh.get_time(), 'enu')
        self.debug_points_pub.publish(msg)

        contour = np.array(bbox_from_rect(
                           rect_from_roi(
                           roi_enclosing_points(self.camera_model, points))), dtype=int)

        sequence = []
        masked_img = None
        while len(sequence) < 20:
            print("whaa")
            img = yield self.front_left_camera_sub.get_next_message()
            img = self.bridge.imgmsg_to_cv2(img)

            mask = contour_mask(contour, img_shape=img.shape)

            img = img[:,:,[2,1,0]]

            masked_img = cv2.bitwise_and(img, img, mask = mask)
            masked_msg = self.bridge.cv2_to_imgmsg(masked_img, "bgr8")
            self.image_debug_pub.publish(masked_msg)

            features = np.array(self.classifier.get_features(img, mask)).reshape(1, 9)
            #print features
            class_probabilities = self.classifier.feature_probabilities(features)[0]
            most_likely_index = np.argmax(class_probabilities)
            most_likely_name = self.classifier.CLASSES[most_likely_index]
            #print most_likely_name
            sequence.append(most_likely_name)
        mode = max(set(sequence), key=sequence.count)
        print('detected as ', mode)
        defer.returnValue((mode, masked_img))

def vec3_to_np(vec):
    return np.array([vec.x, vec.y, vec.z])

def z_filter(db_obj_msg):
    # do a z filter for the led points
    top = max(db_obj_msg.points, key=attrgetter('z')).z
    points = np.array([[i.x, i.y, i.z] for i in db_obj_msg.points 
                        if i.z < top-PANNEL_MAX and i.z > top-PANNEL_MIN])
    return points


def bbox_from_rect(rect):
    bbox = np.array([[rect[0][0], rect[0][1]],
                     [rect[1][0], rect[0][1]],
                     [rect[1][0], rect[1][1]],
                     [rect[0][0], rect[1][1]]])
    return bbox
