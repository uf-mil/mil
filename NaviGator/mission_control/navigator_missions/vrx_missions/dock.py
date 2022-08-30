#!/usr/bin/env python3
import asyncio
import os

import cv2
import numpy as np
import txros
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import DoubleParameter
from image_geometry import PinholeCameraModel
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_tools import pose_to_numpy, rosmsg_to_numpy
from mil_vision_tools.cv_tools import contour_mask, rect_from_roi, roi_enclosing_points
from navigator_vision import VrxStcColorClassifier
from rospkg import RosPack
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Empty
from std_srvs.srv import SetBoolRequest
from tf import transformations
from tf.transformations import quaternion_matrix
from twisted.internet import defer

from .vrx import Vrx

PANNEL_MAX = 0
PANNEL_MIN = 2

CAMERA_LINK_OPTICAL = "wamv/front_left_camera_link_optical"

COLOR_SEQUENCE_SERVICE = "/vrx/scan_dock/color_sequence"

TIMEOUT_SECONDS = 30


class Dock(Vrx):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.classifier = VrxStcColorClassifier()
        self.classifier.train_from_csv()
        self.camera_model = PinholeCameraModel()
        self.rospack = RosPack()

    async def run(self, args):

        self.bridge = CvBridge()

        self.image_debug_pub = self.nh.advertise("/dock_mask_debug", Image)
        await self.image_debug_pub.setup()
        self.init_front_left_camera()
        self.init_front_right_camera()
        args = str.split(args, " ")
        self.color = args[0]
        self.shape = args[1]

        print("entered docking task", self.color, self.shape)

        await self.set_vrx_classifier_enabled(SetBoolRequest(data=False))
        info = await self.front_left_camera_info_sub.get_next_message()
        self.camera_model.fromCameraInfo(info)

        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 10
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        # find dock approach it
        pos = await self.find_dock()

        print("going towards dock")
        await self.move.look_at(pos).set_position(pos).backward(20).go()

        # Decrease cluster tolerance as we approach dock since lidar points are more dense
        # This helps scenario where stc buoy is really close to dock
        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 4
        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])
        await self.nh.sleep(5)

        # get a vector to the longer side of the dock
        dock, pos = await self.get_sorted_objects(name="dock", n=1)
        dock = dock[0]
        position, quat = pose_to_numpy(dock.pose)
        rotation = quaternion_matrix(quat)
        bbox = rosmsg_to_numpy(dock.scale)
        bbox[2] = 0
        max_dim = np.argmax(bbox[:2])
        bbox[max_dim] = 0
        bbox_enu = np.dot(rotation[:3, :3], bbox)
        # this black magic uses the property that a rotation matrix is just a
        # rotated cartesian frame and only gets the vector that points towards
        # the longest side since the vector pointing that way will be at the
        # same index as the scale for the smaller side. This is genius!
        # - Andrew Knee

        # move to first attempt
        print("moving in front of dock")
        goal_pos = None
        curr_pose = await self.tx_pose()
        side_a_bool = False
        side_b_bool = False
        side_a = bbox_enu + position
        side_b = -bbox_enu + position

        # determine which long side is closer to us, go to the closer one
        # (assume VRX peeps are nice and will always have the open side of the dock closer)
        if np.linalg.norm(side_a - curr_pose[0]) < np.linalg.norm(
            side_b - curr_pose[0]
        ):
            print("side_a")
            goal_pos = side_a
            side_a_bool = True
        else:
            print("side_b")
            goal_pos = side_b
            side_b_bool = True

        await self.move.set_position(goal_pos).look_at(position).go()

        # at this moment, we are directly facing the middle of a long side of the dock
        # check if the dock is the correct side.
        await self.nh.sleep(1)
        pixel_diff = await self.dock_checks()

        # if we are on the wrong side, try going to other side of dock
        if pixel_diff is None:
            await self.move.backward(7).go(blind=True, move_type="skid")
            if side_a_bool:
                print("switching to side_b")
                goal_pos = side_b
                side_a_bool = False
                side_b_bool = True
            if side_b_bool:
                print("switching to side_a")
                goal_pos = side_a
                side_b_bool = False
                side_a_bool = True

            await self.move.set_position(goal_pos).look_at(position).go()
            await self.nh.sleep(1)
            pixel_diff = await self.dock_checks()

            if pixel_diff is None:
                print("Could not find any viable options for docking")
                return None

        # do we see any symbols?
        target_symbol = self.color + "_" + self.shape
        symbol_position = await self.get_symbol_position(target_symbol)

        # define how far left and right we want to do
        rot = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

        side_vect = 0.80 * bbox_enu
        if side_a_bool:
            print("creating side a left and right position")
            self.left_position = np.dot(rot, -side_vect) + side_a
            self.right_position = np.dot(rot, side_vect) + side_a
            self.dock_point_left = np.dot(rot, -side_vect) + position
            self.dock_point_right = np.dot(rot, side_vect) + position
        else:
            print("creating side b left and right position")
            self.left_position = np.dot(rot, side_vect) + side_b
            self.right_position = np.dot(rot, -side_vect) + side_b
            self.dock_point_left = np.dot(rot, side_vect) + position
            self.dock_point_right = np.dot(rot, -side_vect) + position

        # if there are symbols, do docking procedure by going to corresponding symbol
        print("The correct docking location is ", symbol_position)

        # position boat in front of correct symbol
        if symbol_position == "left":
            await self.move.set_position(self.left_position).look_at(
                self.dock_point_left
            ).go(blind=True, move_type="skid")
            position = self.dock_point_left
        elif symbol_position == "right":
            await self.move.set_position(self.right_position).look_at(
                self.dock_point_right
            ).go(blind=True, move_type="skid")
            position = self.dock_point_right

        # enter dock
        await self.nh.sleep(1)

        if symbol_position == "foggy":
            print("It seems to be a little foggy")
            await self.dock_fire_undock(foggy=True)
        else:
            await self.dock_fire_undock(foggy=False)

        return True

    # This function is used to see if we see the target symbol in the current image
    async def get_symbol_position(self, target_symbol):

        print("entering get symbol position function")

        target_color, _ = target_symbol.split("_")

        # method = eval('cv2.TM_CCOEFF_NORMED')
        methods = [
            "cv2.TM_CCOEFF",
            "cv2.TM_CCOEFF_NORMED",
            "cv2.TM_CCORR",
            "cv2.TM_CCORR_NORMED",
            "cv2.TM_SQDIFF",
            "cv2.TM_SQDIFF_NORMED",
        ]
        path = self.rospack.get_path("navigator_vision")
        symbol_file = os.path.join(
            path, "datasets/dock_target_images/" + target_symbol + ".png"
        )
        symbol = cv2.imread(symbol_file)
        _, w, h = symbol.shape[::-1]

        for meth in methods:

            # voting system for ten pictures [left, center, right]
            vote = [0, 0, 0]
            foggy_count = 0
            foggy = False

            # loop through ten pictures
            for i in range(10):

                img = await self.front_left_camera_sub.get_next_message()

                img = self.bridge.imgmsg_to_cv2(img)
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                _, width, _ = img.shape[::-1]

                method = eval(meth)
                print("\n")
                print("using ", meth)

                res = cv2.matchTemplate(img, symbol, method)
                _, _, min_loc, max_loc = cv2.minMaxLoc(res)

                # get center pixel of guess
                if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
                    top_left = min_loc
                else:
                    top_left = max_loc

                bottom_right = (top_left[0] + w, top_left[1] + h)
                center_pixel_pos = (
                    (top_left[0] + bottom_right[0]) / 2,
                    (top_left[1] + bottom_right[1]) / 2,
                )

                print(center_pixel_pos[0], center_pixel_pos[1])

                r_comp = img[center_pixel_pos[1]][center_pixel_pos[0]][2]
                g_comp = img[center_pixel_pos[1]][center_pixel_pos[0]][1]
                b_comp = img[center_pixel_pos[1]][center_pixel_pos[0]][0]
                print(r_comp, g_comp, b_comp)

                # check color inside guess
                # if color is what we are looking for, count as a vote
                accept_vote = False
                if (
                    (
                        target_color == "red"
                        and r_comp > 2 * b_comp
                        and r_comp > 2 * g_comp
                    )
                    or (
                        target_color == "blue"
                        and b_comp > 2 * r_comp
                        and b_comp > 2 * g_comp
                    )
                    or (
                        target_color == "green"
                        and g_comp > 2 * r_comp
                        and g_comp > 2 * b_comp
                    )
                    or (
                        target_color == "yellow"
                        and r_comp > 2 * b_comp
                        and g_comp > 2 * b_comp
                    )
                ):
                    accept_vote = True

                if (
                    abs(r_comp - b_comp) < 20
                    and abs(r_comp - g_comp) < 20
                    and abs(b_comp - g_comp) < 20
                    and r_comp > 160
                ):
                    foggy_count += 1

                cv2.rectangle(img, top_left, bottom_right, 255, 2)
                masked_msg = self.bridge.cv2_to_imgmsg(img, "bgr8")
                self.image_debug_pub.publish(masked_msg)

                if accept_vote:
                    if max_loc[0] < width / 3.0:
                        vote[0] = vote[0] + 1
                    elif width / 3.0 < max_loc[0] < 2 * width / 3.0:
                        vote[1] = vote[1] + 1
                    else:
                        vote[2] = vote[2] + 1

            if foggy_count > 5:
                foggy = True
                break

            # check if we have enough up votes on the maximum choice
            most_likely_index = np.argmax(vote)
            if vote[most_likely_index] > 5:
                break

        if foggy:
            return "foggy"

        symbol_position = None
        if vote[0] <= 5 and vote[1] <= 5 and vote[2] <= 5:
            return symbol_position

        if most_likely_index == 0:
            symbol_position = "left"
        elif most_likely_index == 1:
            symbol_position = "center"
        elif most_likely_index == 2:
            symbol_position = "right"

        return symbol_position

    # This function is used to help aim_and_fire you know... aim
    async def get_black_square_center(self, foggy=False):

        img = await self.front_right_camera_sub.get_next_message()
        img = self.bridge.imgmsg_to_cv2(img)
        image = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        _, width, height = image.shape[::-1]

        if foggy:
            print("using foggy threshold")
            value = 140
        else:
            value = 25

        # set bounds for finding only black objects
        lower = np.array([0, 0, 0], dtype="uint8")
        upper = np.array([255, 10, value], dtype="uint8")
        mask = cv2.inRange(image, lower, upper)

        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # guess location of center of black square in case we can't find it
        center_pixel_col = 450
        center_pixel_row = 400
        h = 75
        w = 75

        # delete contours that don't meet criteria of big black square
        indices_to_delete = []
        for i, v in enumerate(cnts):
            x, y, w, h = cv2.boundingRect(v)

            if abs(w - h) > 40:
                # print("removing due to difference")
                indices_to_delete.append(i)
                continue
            if (w < 90) or (h < 90):
                # print("removing due to size")
                indices_to_delete.append(i)
                continue

            print("width and height: ", w, h)
            print(x)
            print(y)
            print(w)
            print(h)
            center_pixel_row = y
            center_pixel_col = x

        for index in indices_to_delete[::-1]:
            cnts.pop(index)

        print("The new size of cnts is: ", len(cnts))
        if len(cnts) == 0:
            return None

        # get the center pixel of the black square
        # center_pixel_row and center_pixel_col are defined as
        # the top left corner of the contour
        if center_pixel_row + h / 2 < height:
            center_pixel_row = center_pixel_row + h / 2
        else:
            center_pixel_row = height - 1

        if center_pixel_col + w / 2 < width:
            center_pixel_col = center_pixel_col + w / 2
        else:
            center_pixel_col = width - 1

        symbol_position = [center_pixel_row, center_pixel_col]

        cv2.rectangle(mask, (x, y), (x + w, y + h), 125, 2)
        cv2.rectangle(mask, (425, 400), (525, 480), 150, 2)

        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.image_debug_pub.publish(mask_msg)

        return symbol_position

    # this function moves the boat until it is aiming at the big black square
    # once the boat is lined up, it fires the balls
    async def aim_and_fire(self, foggy=False):

        for i in range(3):

            # obtain the pixel position of the small black square
            square_pix = await self.get_black_square_center(foggy=foggy)

            # ensure boat is lined up to be able to hit the target
            # by making sure the black box is in correct part of image
            # values were obtained by setting the ball shooter at a
            # specific yaw and pitch and determining where the box needed
            # to be for the ball to go in the box

            if square_pix is None:
                print("square pix is none")
                continue

            print(square_pix)

            min_x = 425
            max_x = 525
            mid_x = (min_x + max_x) / 2
            min_y = 400
            max_y = 480

            # calculated from pixel size of small square and 0.25m
            # Note this is only valid given the distance of the boat
            # relative to the dock images at this moment in the course
            pixel_to_meter = 350.0

            # if target is too far left, adjust left (otherwise ball will pull right)
            #   Note: if ball is missing right, shift range right
            # if target is too far right, adjust right (otherwise ball will pull left)
            #   Note: if ball is missing left, shift range left
            if square_pix[1] < min_x:
                print("adjusting left")
                print(square_pix)
                adjustment = (mid_x - square_pix[1]) / pixel_to_meter
                print("Adjustment: ", adjustment)
                await self.move.left(adjustment).go(blind=True, move_type="skid")
            elif square_pix[1] > max_x:
                print("adjusting right")
                print(square_pix)
                adjustment = (square_pix[1] - mid_x) / pixel_to_meter
                print("Adjustment: ", adjustment)
                await self.move.right(adjustment).go(blind=True, move_type="skid")

            await self.nh.sleep(1)

        # loop here to double check that box is still in place in case of crazy waves
        #   if undershooting, shift range right, if overshooting, shift range left
        while True:
            square_pix = await self.get_black_square_center(foggy=foggy)
            if square_pix is None:
                print("square pix is none")
                continue

            print(square_pix)
            if square_pix[0] < min_y or square_pix[1] < min_x:
                print("Aim is too low/right")
            elif square_pix[0] > max_y or square_pix[1] > max_x:
                print("Aim is too high/left")
            else:
                break

        self.fire_ball.publish(Empty())

    # This function will tell us if we are on the correct side of the dock.
    # It will also tell us (assuming we are on the correct side of the dock)
    # how many pixels (left or right) the center of the docking area is.
    async def dock_checks(self):

        # obtain one image
        img = await self.front_left_camera_sub.get_next_message()
        img = self.bridge.imgmsg_to_cv2(img)
        hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
        _, width, height = hsv_img.shape[::-1]

        # mask for only blue (to get only the water for the most part)
        lower = np.array([100, 50, 50], dtype="uint8")
        upper = np.array([125, 255, 255], dtype="uint8")
        mask = cv2.inRange(hsv_img, lower, upper)

        base_of_dock = None
        left_wall = None
        right_wall = None

        # find base of dock starting from the bottom center of image to ensure we are facing correct side of dock
        for i in range(height):
            if mask[(height - i - 1), (width / 2)] == 0:
                print("we found the base of the dock")
                base_of_dock = height - i - 1
                break

        # if we hit something not blue before reaching halfway through the image, we are on the wrong side
        if base_of_dock > height / 2:
            print("we are on wrong side")
            defer.returnValue(None)

        # find left wall of docking area
        for i in range(width / 2):

            # look to left of pixel until we hit black
            if mask[(base_of_dock + 20), (width / 2 - i)] == 0:
                left_wall = width / 2 - i
                print("Left pixel of docking area is: ", left_wall)
                break

            if i == width / 2 - 1:
                print("there is no left side of docking area")
                return None

        # find right wall of docking area
        for i in range(width / 2):

            # look to right of pixel until we hit black
            if mask[(base_of_dock + 20), (width / 2 + i)] == 0 and i != 0:
                right_wall = width / 2 + i
                print("Right pixel of docking area is: ", right_wall)
                break

            if i == width / 2 - 1:
                print("there is no right side of docking area")
                return None

        midpoint = (left_wall + right_wall) / 2

        # helpful for debugging to ensure we truly found the center of the docking area
        for i in range(height):
            mask[i, midpoint] = 0

        # send out debugging image
        mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
        self.image_debug_pub.publish(mask_msg)

        # return that we are on the correct side and the difference in pixels between
        # midpoint of docking area and center of image
        print("This is the pixel diff: ", width / 2 - midpoint)
        return width / 2 - midpoint

    # This function is used to:
    #   - center the boat before docking (or at least try to)
    #   - dock the boat
    #   - shoot the balls at the black square
    #   - undock
    async def dock_fire_undock(self, foggy=False):

        pixel_diff = await self.dock_checks()

        if pixel_diff is None:
            print("something is wrong")
            return False

        # magic number that determines how far left or right we should move so that we can center
        # calculated from average pixel width of docking area / width of docking area in meters
        pixel_to_meter = 80.0  # (340/4)
        adjustment = pixel_diff / pixel_to_meter

        if adjustment > 0:
            print("adjusting left", adjustment)
            await self.nh.sleep(1)
            await self.move.left(adjustment).go(blind=True, move_type="skid")
        elif pixel_diff < 0:
            print("adjusting right", adjustment)
            await self.nh.sleep(1)
            await self.move.right(abs(adjustment)).go(blind=True, move_type="skid")

        # dock the boat
        await self.move.forward(7).go(blind=True, move_type="skid")

        # fire ball
        print("Aim and Fire!")
        await self.nh.sleep(1)
        for i in range(4):
            try:
                await txros.util.wrap_timeout(
                    self.aim_and_fire(foggy=foggy), 15, "Trying to shoot"
                )
            except asyncio.TimeoutError:
                print("Let's just take the shot anyways")
                self.fire_ball.publish(Empty())

        # Exit dock
        await self.move.backward(7).go(blind=True, move_type="skid")

        await self.send_feedback("Done!")

        return True

    # This function is used to find the position of the dock at the beginning of this mission
    async def find_dock(self):

        msgs = None
        while msgs is None:
            try:
                msgs, poses = await self.get_sorted_objects(name="UNKNOWN", n=-1)
            except Exception as e:
                await self.move.forward(10).go()
        await self.pcodar_label(msgs[0].id, "dock")
        # if no pcodar objects, throw error, exit mission
        pose = poses[0]

        return pose

    # Potentially deprecated
    async def prepare_for_docking(self):
        # This function looks at the two squares in front of the boat
        # and it gets the middle pixel between the two squares.
        # If the middle pixel is for some reason not in the middle of our camera...
        # adjust the boat postiion before docking
        print("prepare for landing!")

        img = await self.front_left_camera_sub.get_next_message()
        img = self.bridge.imgmsg_to_cv2(img)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        _, width, height = img.shape[::-1]
        stencil = np.zeros(img.shape).astype(img.dtype)

        fill_color = [255, 255, 255]  # any BGR color value to fill with
        mask_value = 255  # 1 channel white (can be any non-zero uint8 value)

        # create custom contour
        top_left = [width / 3.0, 0]
        bottom_right = [2 * width / 3.0, height / 5.0]
        contours = [
            np.array(
                [
                    [top_left[0], top_left[1]],
                    [top_left[0], bottom_right[1]],
                    [bottom_right[0], bottom_right[1]],
                    [bottom_right[0], top_left[1]],
                ],
                dtype=np.int32,
            )
        ]

        stencil = np.zeros(img.shape[:-1]).astype(np.uint8)
        cv2.fillPoly(stencil, contours, mask_value)

        sel = stencil != mask_value  # select everything that is not mask_value
        img[sel] = fill_color  # and fill it with fill_color

        lower = np.array([0, 0, 0], dtype="uint8")
        upper = np.array([20, 20, 20], dtype="uint8")
        mask = cv2.inRange(img, lower, upper)
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[0] if len(cnts) == 2 else cnts[1]

        # Remove any other noise that aren't the squares we want
        indices_to_delete = []
        for i, v in enumerate(cnts):
            x, y, w, h = cv2.boundingRect(v)
            print(w, h)
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
            # self.image_debug_pub.publish(masked_msg)

            # assume there are only two contours (hopefully, otherwise, make contour and mask tighter)
            big_square_x, _, w, h = cv2.boundingRect(cnts[0])
            small_square_x, _, _, _ = cv2.boundingRect(cnts[1])
            middle_of_squares_x = (big_square_x + small_square_x) / 2
            middle_of_image = width / 2
            pixel_diff = abs(middle_of_image - middle_of_squares_x)
            pixel_to_meter = (
                w / 0.5
            )  # width of big square over size of side length in m
            adjustment = pixel_diff / pixel_to_meter

            print("number of contours: ", len(cnts))
            print("pixel diff: ", pixel_diff)
            print("adjustment: ", adjustment)
            print("square width: ", w)
            print("square height: ", h)

            if middle_of_squares_x > middle_of_image:
                print("adjusting right")
                await self.move.right(adjustment).go(blind=True, move_type="skid")
            elif middle_of_squares_x < middle_of_image:
                print("adjusting left")
                await self.move.left(adjustment).go(blind=True, move_type="skid")
