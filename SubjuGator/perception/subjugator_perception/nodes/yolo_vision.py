#!/usr/bin/env python3
import rospy
from mil_ros_tools import (
    Image_Subscriber,
)
from vision_stack import (
    ObjectDetectionLayer,
    ResizeLayer,
    RGBtoBGRLayer,
    VisionStack,
)

__author__ = "Daniel Parra"


class YoloVision:
    def __init__(self):
        path_to_weights = "/home/zobelisk/mil_weights/last.pt"
        class_names = [
            "Bin",
            "Bin blue",
            "Bin red",
            "Blue spiral",
            "Coral",
            "Nautilus",
            "Path Marker",
            "Red buoy",
            "Red spiral",
            "Tube Worm",
        ]
        class_colors = [
            (100, 50, 70),
            (200, 50, 60),
            (70, 180, 20),
            (255, 0, 0),
            (255, 255, 0),
            (150, 255, 0),
            (56, 200, 150),
            (255, 0, 255),
            (0, 0, 255),
            (150, 0, 0),
        ]
        camera = rospy.get_param("~image_topic", "/camera/front/right/image_color")
        self.vs = VisionStack(
            layers=[
                ResizeLayer(960, 608),
                RGBtoBGRLayer(),
                ObjectDetectionLayer(
                    path_to_weights,
                    0.5,
                    0.5,
                    class_names,
                    class_colors,
                    True,
                ),
                RGBtoBGRLayer(),
            ],
        )
        self.image_sub = Image_Subscriber(camera, self.detection_callback)

    def detection_callback(self, msg):
        self.vs.run(msg, True)


if __name__ == "__main__":
    rospy.init_node("yolo_detections")
    YoloVision()
    rospy.spin()
