#!/usr/bin/env python
from re import T
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
        self.init_front_right_camera()
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

        #determine which long side is closer to us, go to the closer one
        #(assume VRX peeps are nice and will always have the open side of the dock closer)
        if np.linalg.norm(side_a - curr_pose[0]) < np.linalg.norm(side_b - curr_pose[0]):
            print("side_a")
            goal_pos = side_a
            side_a_bool = True
        else:
            print("side_b")
            goal_pos = side_b
            side_b_bool = True

        yield self.move.set_position(goal_pos).look_at(position).go()

        target_symbol = self.color + "_" + self.shape
        symbol_position = yield self.get_symbol_position(target_symbol)
        print("The correct docking location is ", symbol_position)

        ###
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

        side_vect = 0.80 * bbox_enu
        if side_a_bool:
            print("creating side a left and right position")
            left_position = np.dot(rot, -side_vect) + side_a
            right_position = np.dot(rot, side_vect) + side_a
            dock_point_left = np.dot(rot, -side_vect) + position
            dock_point_right = np.dot(rot, side_vect) + position
        else:
            print("creating side b left and right position")
            left_position = np.dot(rot, side_vect) + side_b
            right_position = np.dot(rot, -side_vect) + side_b
            dock_point_left = np.dot(rot, side_vect) + position
            dock_point_right = np.dot(rot, -side_vect) + position
        ###        

        #position boat in front of correct symbol
        if symbol_position == "left":
            yield self.move.set_position(left_position).look_at(dock_point_left).go(blind=True, move_type="skid")
            position = dock_point_left
        elif symbol_position == "right":
            yield self.move.set_position(right_position).look_at(dock_point_right).go(blind=True, move_type="skid")
            position = dock_point_right

        #enter dock
        yield self.nh.sleep(1)
        yield self.prepare_for_docking()
        yield self.move.forward(6.5).go(blind=True, move_type="skid")

        #fire ball
        yield self.nh.sleep(1)
        for i in range(4):
            yield self.aim_and_fire()

        #Exit dock
        yield self.move.backward(7).go(blind=True, move_type="skid")

        yield self.send_feedback('Done!')


    @txros.util.cancellableInlineCallbacks
    def get_symbol_position(self, target_symbol):
        
        target_color,_ = target_symbol.split('_')

        #method = eval('cv2.TM_CCOEFF_NORMED')
        methods = ['cv2.TM_CCOEFF', 'cv2.TM_CCOEFF_NORMED', 'cv2.TM_CCORR',
            'cv2.TM_CCORR_NORMED', 'cv2.TM_SQDIFF', 'cv2.TM_SQDIFF_NORMED']
        path = self.rospack.get_path('navigator_vision')
        symbol_file = os.path.join(path, 'datasets/dock_target_images/' + target_symbol + ".png")
        symbol = cv2.imread(symbol_file)
        _,w,h = symbol.shape[::-1]

        for meth in methods:

            #voting system for ten pictures [left, center, right]
            vote = [0,0,0]

            #loop through ten pictures
            for i in range(10):

                img = yield self.front_left_camera_sub.get_next_message()

                img = self.bridge.imgmsg_to_cv2(img)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                _,width,_ = img.shape[::-1]

                method = eval(meth)
                print("\n")
                print("using ", meth)

                res = cv2.matchTemplate(img,symbol,method)
                _, _, min_loc, max_loc = cv2.minMaxLoc(res)

                #get center pixel of guess
                if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                    top_left = min_loc
                else:
                    top_left = max_loc

                bottom_right = (top_left[0] + w, top_left[1] + h)
                center_pixel_pos = ((top_left[0] + bottom_right[0]) / 2, (top_left[1] + bottom_right[1]) / 2)

                print(center_pixel_pos[0], center_pixel_pos[1])

                r_comp = img[ center_pixel_pos[1] ][ center_pixel_pos[0] ][2]
                g_comp = img[ center_pixel_pos[1] ][ center_pixel_pos[0] ][1]
                b_comp = img[ center_pixel_pos[1] ][ center_pixel_pos[0] ][0]
                print(r_comp,g_comp,b_comp)


                #check color inside guess
                #if color is what we are looking for, count as a vote
                accept_vote = False
                if (target_color == "red" and r_comp > 20 and b_comp < 5 and g_comp < 5) or \
                   (target_color == "blue" and r_comp < 5 and b_comp > 20 and g_comp < 5) or \
                   (target_color == "green" and r_comp < 5 and b_comp < 5 and g_comp > 20) or \
                   (target_color == "yellow" and r_comp > 20 and b_comp < 5 and g_comp > 20):
                    accept_vote = True

                cv2.rectangle(img,top_left, bottom_right, 255, 2)
                masked_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.image_debug_pub.publish(masked_msg)

                if accept_vote:
                    if max_loc[0] < width/3.0:
                        vote[0] = vote[0] + 1
                    elif width/3.0 < max_loc[0] < 2*width/3.0:
                        vote[1] = vote[1] + 1
                    else:
                        vote[2] = vote[2] + 1 

            #check if we have enough up votes on the maximum choice
            most_likely_index = np.argmax(vote)
            if vote[most_likely_index] > 5:
                break

        symbol_position = "left"
        if most_likely_index == 0:
            symbol_position = "left"
        elif most_likely_index == 1:
            symbol_position = "center"
        elif most_likely_index == 2:
            symbol_position = "right"

        defer.returnValue(symbol_position)

    @txros.util.cancellableInlineCallbacks
    def prepare_for_docking(self):
        #This function looks at the two squares in front of the boat
        #and it gets the middle pixel between the two squares.
        #If the middle pixel is for some reason not in the middle of our camera...
        #adjust the boat postiion before docking
        print("prepare for landing!")

        img = yield self.front_left_camera_sub.get_next_message()
        img = self.bridge.imgmsg_to_cv2(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        _,width,height = img.shape[::-1]
        stencil = np.zeros(img.shape).astype(img.dtype)

        fill_color = [255, 255, 255] # any BGR color value to fill with
        mask_value = 255            # 1 channel white (can be any non-zero uint8 value)

        #create custom contour
        top_left = [width/3.0, 0]
        bottom_right = [2*width/3.0, height/5.0]
        contours = [np.array([[top_left[0],top_left[1]], [top_left[0],bottom_right[1]], [bottom_right[0],bottom_right[1]], [bottom_right[0],top_left[1]]], dtype=np.int32)]

        stencil  = np.zeros(img.shape[:-1]).astype(np.uint8)
        cv2.fillPoly(stencil, contours, mask_value)

        sel      = stencil != mask_value # select everything that is not mask_value
        img[sel] = fill_color            # and fill it with fill_color

        lower = np.array([0, 0, 0], dtype="uint8")
        upper = np.array([20, 20, 20], dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        #Remove any other noise that aren't the squares we want
        indices_to_delete = []
        for i,v in enumerate(cnts):
            x,y,w,h = cv2.boundingRect(v)
            print(w,h)
            print("Length: ", len(cnts))
            if (w > (h + 5)) or (w < (h - 5)):
                print("removing due to difference")
                indices_to_delete.append(i)
                continue

            if (w < 20) or (w > 60) or (h < 20) or (h > 60):
                print("removing due to size")
                indices_to_delete.append(i)
                continue
            print("Accepted")

        for index in indices_to_delete[::-1]:
            cnts.pop(index)

        print("The new size of cnts is: ", len(cnts))
        if len(cnts) == 2:
            
            masked_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
            self.image_debug_pub.publish(masked_msg)

            #assume there are only two contours (hopefully, otherwise, make contour and mask tighter)
            big_square_x,_,w,h = cv2.boundingRect(cnts[0])
            small_square_x,_,_,_ = cv2.boundingRect(cnts[1])
            middle_of_squares_x = (big_square_x + small_square_x) / 2
            middle_of_image = width/2
            pixel_diff = abs(middle_of_image - middle_of_squares_x)
            pixel_to_meter = w / 0.5 #width of big square over size of side length in m
            adjustment = pixel_diff / pixel_to_meter

            print("number of contours: ", len(cnts))
            print("pixel diff: ", pixel_diff)
            print("adjustment: ", adjustment)
            print("square width: ", w)
            print("square height: ", h)

            if middle_of_squares_x > middle_of_image:
                print("adjusting right")
                yield self.move.right(adjustment).go(blind=True, move_type="skid")
            elif middle_of_squares_x < middle_of_image:
                print("adjusting left")
                yield self.move.left(adjustment).go(blind=True, move_type="skid")

    @txros.util.cancellableInlineCallbacks
    def get_black_square_center(self):

        img = yield self.front_right_camera_sub.get_next_message()
        img = self.bridge.imgmsg_to_cv2(img)
        image = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        _,width,height = image.shape[::-1]

        #set bounds for finding only black objects
        lower = np.array([0, 0, 0], dtype="uint8")
        upper = np.array([20, 20, 20], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]
        cv2.fillPoly(mask, cnts, (255,255,255))

        #guess location of center of black square in case we can't find it
        center_pixel_col = 450
        center_pixel_row = 400

        for c in cnts:
            x,y,w,h = cv2.boundingRect(c)
            print("width and height: ", w,h)
            if w > 95 and h > 95:
                print(x)
                print(y)
                print(w)
                print(h)
                center_pixel_row = y
                center_pixel_col = x
                break
            else:
                continue
                
        #get the center pixel of the black square
        if center_pixel_row + h / 2 < height:
            center_pixel_row = center_pixel_row + h / 2
        if center_pixel_col + w / 2 < width:
            center_pixel_col = center_pixel_col + w / 2

        symbol_position = [center_pixel_row, center_pixel_col]

        cv2.rectangle(mask,(x,y), (x + w, y + h), 255, 2)
        cv2.rectangle(mask,(425,390), (525,470), 150, 2)
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.image_debug_pub.publish(mask_msg)

        defer.returnValue(symbol_position)

    @txros.util.cancellableInlineCallbacks
    def aim_and_fire(self):

        for i in range(3):

            #obtain the pixel position of the small black square
            square_pix = yield self.get_black_square_center()

            #ensure boat is lined up to be able to hit the target
            #by making sure the black box is in correct part of image
            #values were obtained by setting the ball shooter at a
            #specific yaw and pitch and determing where the box needed
            #to be for the ball to go in the box

            print(square_pix)

            min_x = 425
            max_x = 525
            mid_x = (min_x + max_x) / 2
            min_y = 390
            max_y = 470
            
            print(square_pix)

            #calculated from pixel size of small square and 0.25m
            #Note this is only valid given the distance of the boat
            #relative to the dock images at this moment in the course
            pixel_to_meter = 360.0

            #if target is too far left, adjust left (otherwise ball will pull right)
            #   Note: if ball is missing right, shift range right
            #if target is too far right, adjust right (otherwise ball will pull left)
            #   Note: if ball is missing left, shift range left
            if square_pix[1] < min_x:
                print("adjusting left")
                print(square_pix)
                adjustment = (mid_x - square_pix[1]) / pixel_to_meter
                print("Adjustment: ", adjustment)
                yield self.move.left(adjustment).go(blind=True, move_type="skid")
            elif square_pix[1] > max_x:
                print("adjusting right")
                print(square_pix)
                adjustment = (square_pix[1] - mid_x) / pixel_to_meter
                print("Adjustment: ", adjustment)
                yield self.move.right(adjustment).go(blind=True, move_type="skid")

            yield self.nh.sleep(0.5)

        #loop here to double check that box is still in place in case of crazy waves
        #   if undershooting, shift range right, if overshooting, shift range left
        square_pix = yield self.get_black_square_center()
        while True:
            square_pix = yield self.get_black_square_center()
            print(square_pix)
            if square_pix[0] < min_y or square_pix[1] < min_x:
                print("Aim is too low/right")
            elif square_pix[0] > max_y or square_pix[1] > max_x:
                print("Aim is too high/left")
            else:
                break

        self.fire_ball.publish(Empty())

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
