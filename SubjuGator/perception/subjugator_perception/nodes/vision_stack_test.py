#!/usr/bin/env python3
import rospy
from image_geometry import PinholeCameraModel
from mil_ros_tools import (
    Image_Subscriber,
)
from vision_stack import ResizeLayer, UnderWaterImageEnhancementLayer, VisionStack

__author__ = "Daniel Parra"


class ObjectDetectionTest:
    def __init__(self):
        camera = rospy.get_param("~image_topic", "/camera/front/right/image_rect_color")
        SIZE = (960, 608)
        self.vs = VisionStack(
            layers=[
                ResizeLayer((0, 0), 960, 608),
                UnderWaterImageEnhancementLayer(SIZE),
            ],
            input_size=SIZE,
        )

        self.image_sub = Image_Subscriber(camera, self.detection_callback)
        self.camera_info = self.image_sub.wait_for_camera_info()
        assert self.camera_info is not None
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)

    def detection_callback(self, msg):
        # Create Image from array
        self.vs.run(msg, True)


if __name__ == "__main__":
    rospy.init_node("vision_pipeline_test")
    ObjectDetectionTest()
    rospy.spin()
