#!/usr/bin/env python3
import numpy as np
from enum import Enum
from geometry_msgs.msg import Point
from mil_misc_tools import ThrowingArgumentParser
from mil_msgs.msg import ObjectsInImage
from mil_msgs.srv import CameraToLidarTransform, CameraToLidarTransformRequest
from mil_tools import rosmsg_to_numpy

from std_srvs.srv import SetBoolRequest

from .navigator import NaviGatorMission

class MoveState(Enum):
    NOT_STARTED = 1
    RUNNING = 2
    CANCELLED = 3
    FINISHED = 4

class Wildlife2024(NaviGatorMission):
    animals_observed = {
        "B_M" : False,    # Manatee => Counter clockwise
        "G_I" : False,    # Iguana => Clockwise (by choice)
        "R_P" : False     # Python => Clockwise
    }

    @classmethod
    async def setup(cls):
        cls.camsub = cls.nh.subscribe("/bbox_pub", ObjectsInImage)
        await cls.camsub.setup()

        cls.camera_lidar_tf = cls.nh.get_service_client(
            "/wamv/sensors/camera/front_right_cam/image_raw",
            CameraToLidarTransform,
        )
    
    @classmethod
    async def shutdown(cls):
        await cls.camsub.shutdown()

    async def run(self, args):
        # Check nearest objects

        # Subscribe to the camera topic

        # Gat access to PCODAR service
        unknown_objects = await self.get_sorted_objects(name="UNKNOWN", n=3, throw=False)

        # Associate Object to class
        
            # Get the point cloud points associated with the object

            # Map the points to the camera frame

            # Classify by mode OR mean of frequent pixels 
        
        # Go to buoy and circle accordingly
        pass
