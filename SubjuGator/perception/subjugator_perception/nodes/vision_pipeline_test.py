#!/usr/bin/env python3
import rospy
from image_geometry import PinholeCameraModel
from mil_ros_tools import (
    Image_Subscriber,
)
from yoloros import Detector

# from vision_stack import VisionStack

__author__ = "Daniel Parra"


class ObjectDetectionTest:
    def __init__(self):
        camera = rospy.get_param("~image_topic", "/camera/front/left/image_rect_color")
        self.detector = Detector("robosub24", device="cpu")

        self.image_sub = Image_Subscriber(camera, self.detection_callback)
        self.camera_info = self.image_sub.wait_for_camera_info()
        assert self.camera_info is not None
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)

    def detection_callback(self, msg):
        # Create Image from array
        print("Detecting...")
        self.detector.display_detection_ros_msg(msg, conf_thres=0.85)


if __name__ == "__main__":
    rospy.init_node("vision_pipeline_test")
    ObjectDetectionTest()
    rospy.spin()
