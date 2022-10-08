#!/usr/bin/env python3
import asyncio
from operator import attrgetter

import numpy as np
import txros
from cv2 import bitwise_and
from cv_bridge import CvBridge
from dynamic_reconfigure.msg import DoubleParameter
from image_geometry import PinholeCameraModel
from mil_msgs.srv import ObjectDBQuery, ObjectDBQueryRequest
from mil_tools import numpy_to_pointcloud2 as np2pc2
from mil_tools import rosmsg_to_numpy
from mil_vision_tools.cv_tools import contour_mask, rect_from_roi, roi_enclosing_points
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import SetBoolRequest
from vrx_gazebo.srv import ColorSequence, ColorSequenceRequest

from .navigator import Navigator

LED_PANEL_MAX = 0.1
LED_PANEL_MIN = 0.5

STC_HEIGHT = 2.3
STC_WIDTH = 2

CAMERA_LINK_OPTICAL = "wamv/front_left_cam_link_optical"

COLOR_SEQUENCE_SERVICE = "/vrx/scan_dock/color_sequence"

TIMEOUT_SECONDS = 120

COLORS = ["red", "green", "black", "blue"]


class ScanTheCode(Navigator):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.camera_model = PinholeCameraModel()

    @classmethod
    async def shutdown(cls):
        await cls.debug_points_pub.shutdown()
        await cls.image_debug_pub.shutdown()

    async def run(self, args):
        self.debug_points_pub = self.nh.advertise("/stc_led_points", PointCloud2)
        self.bridge = CvBridge()
        self.image_debug_pub = self.nh.advertise("/stc_mask_debug", Image)

        await asyncio.gather(
            self.debug_points_pub.setup(),
            self.image_debug_pub.setup(),
        )

        await self.change_wrench("/wrench/autonomous")
        await self.set_classifier_enabled(SetBoolRequest(data=False))
        info = await self.front_left_camera_info_sub.get_next_message()
        self.camera_model.fromCameraInfo(info)

        pcodar_cluster_tol = DoubleParameter()
        pcodar_cluster_tol.name = "cluster_tolerance_m"
        pcodar_cluster_tol.value = 20

        await self.pcodar_set_params(doubles=[pcodar_cluster_tol])

        # Try to find the stc light
        try:
            pose = await self.find_stc()
        except Exception as e:
            sequence = ["red", "green", "blue"]
            await self.report_sequence(sequence)
            return sequence

        # Go to the stc light
        await self.move.look_at(pose).set_position(pose).backward(5).go()
        await self.nh.sleep(5)
        # get updated points and tf now that we a closer
        stc_query = await self.get_sorted_objects(name="stc_platform", n=1)
        stc = stc_query[0][0]
        print("Getting transform")
        tf = await self.tf_listener.get_transform(CAMERA_LINK_OPTICAL, "enu")
        print("Transform gotted!")
        points = z_filter(stc)

        msg = np2pc2(points, self.nh.get_time(), "enu")
        self.debug_points_pub.publish(msg)

        print("debug points have been published")
        points = np.array([tf.transform_point(points[i]) for i in range(len(points))])

        print("obtaining contour")
        contour = np.array(
            bbox_from_rect(
                rect_from_roi(roi_enclosing_points(self.camera_model, points))
            ),
            dtype=int,
        )
        try:
            sequence = await txros.util.wrap_timeout(
                self.get_sequence(contour), TIMEOUT_SECONDS
            )
        except asyncio.TimeoutError:
            sequence = ["red", "green", "blue"]
        print("Scan The Code Color Sequence", sequence)

        self.send_feedback("Done!")
        return sequence

    async def get_sequence(self, contour):
        sequence = []
        print("GETTING SEQUENCE")
        while len(sequence) < 3:

            img = await self.front_left_camera_sub.get_next_message()
            img = self.bridge.imgmsg_to_cv2(img)

            mask = contour_mask(contour, img_shape=img.shape)

            img = img[:, :, [2, 1, 0]]
            mask_msg = self.bridge.cv2_to_imgmsg(
                bitwise_and(img, img, mask=mask), "bgr8"
            )

            print("PUBLISHING MASK")
            self.image_debug_pub.publish(mask_msg)

            print("WAITING FOR STC BOUNDING BOX")
            bounding_box_msg = await self.stc_objects.get_next_message()
            if len(bounding_box_msg.detections) == 0:
                print("Nothing Detected")
                continue
            print("STC BOUNDING BOX FOUND")

            ##############

            most_likely_name = COLORS[bounding_box_msg.detections[0].results[0].id]

            if most_likely_name == "black":
                sequence = []
            elif sequence == [] or most_likely_name != sequence[-1]:
                sequence.append(most_likely_name)

            print(sequence)

        return sequence

    async def report_sequence(self, sequence):
        color_sequence = ColorSequenceRequest()
        color_sequence.color1 = sequence[0]
        color_sequence.color2 = sequence[1]
        color_sequence.color3 = sequence[2]

        # try:
        #    await self.sequence_report(color_sequence)
        # except Exception as e: #catch error in case vrx scroing isn't running
        #    print(e)

    async def find_stc(self):
        pose = None
        # see if we already got scan the code tower
        try:
            _, poses = await self.get_sorted_objects(name="stc_platform", n=1)
            pose = poses[0]
        # in case stc platform not already identified
        except Exception as e:
            # get all pcodar objects
            try:
                _, poses = await self.get_sorted_objects(name="UNKNOWN", n=-1)
            # if no pcodar objects, drive forward
            except Exception as e:
                await self.move.forward(50).go()
                # get all pcodar objects
                _, poses = await self.get_sorted_objects(name="UNKNOWN", n=-1)
                # if still no pcodar objects, guess RGB and exit mission
            # go to nearest obj to get better data on that obj
            print("going to nearest object")
            await self.move.set_position(poses[0]).go()
            # get data on closest obj
            msgs, poses = await self.get_sorted_objects(name="UNKNOWN", n=1)
            if np.linalg.norm(rosmsg_to_numpy(msgs[0].scale)) > 6.64:
                # much bigger than scale of stc
                # then we found the dock
                await self.pcodar_label(msgs[0].id, "dock")
                # get other things
                msgs, poses = await self.get_sorted_objects(name="UNKNOWN", n=1)
                # if no other things, throw error and exit mission
                await self.pcodar_label(msgs[0].id, "stc_platform")
                pose = poses[0]
            else:  # if about same size as stc, label it stc
                await self.pcodar_label(msgs[0].id, "stc_platform")
                pose = poses[0]
        return pose


def z_filter(db_obj_msg):
    # do a z filter for the led points
    top = max(db_obj_msg.points, key=attrgetter("z")).z

    # Define the height where we expect to find the stc buoy light relative to the top of the buoy
    points = np.array(
        [
            [i.x, i.y, i.z]
            for i in db_obj_msg.points
            if i.z < top - LED_PANEL_MAX and i.z > top - LED_PANEL_MIN
        ]
    )
    return points


def bbox_from_rect(rect):
    print("This is the rect: ", rect)
    bbox = np.array(
        [
            [rect[0][0] - 20, rect[0][1] - rect[0][1]],
            [rect[1][0] + 20, rect[0][1] - rect[0][1]],
            [rect[1][0] + 20, rect[1][1] + 20],
            [rect[0][0] - 20, rect[1][1] + 20],
        ]
    )
    return bbox
