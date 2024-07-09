#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from mil_ros_tools import (
    Image_Subscriber,
)
from vision_stack import (
    CustomLayer,
    ObjectDetectionLayer,
    ResizeLayer,
    VisionStack,
)

__author__ = "Daniel Parra"


def add_blue_tint(image, intensity=0.2):
    blue = np.full(image.shape, (255, 0, 0), dtype=np.uint8)
    return cv2.addWeighted(image, 1 - intensity, blue, intensity, 0)


def blur_image(image, kernel_size=(15, 15)):
    return cv2.GaussianBlur(image, kernel_size, 0)


def add_ripples(image, ripple_strength=5, frequency=20):
    rows, cols, _ = image.shape
    ripple_map_x = np.zeros((rows, cols), dtype=np.float32)
    ripple_map_y = np.zeros((rows, cols), dtype=np.float32)

    for i in range(rows):
        for j in range(cols):
            ripple_map_x[i, j] = j + ripple_strength * np.sin(2 * np.pi * i / frequency)
            ripple_map_y[i, j] = i + ripple_strength * np.sin(2 * np.pi * j / frequency)

    return cv2.remap(image, ripple_map_x, ripple_map_y, interpolation=cv2.INTER_LINEAR)


def make_underwater_effect(image, *args):
    if len(image.shape) == 2: # Grayscale image (height, width)
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
    image = add_blue_tint(image)
    image = blur_image(image)
    return (image, None)


class YoloVision:
    def __init__(self):
        path_to_weights = "/home/sub8/ml_weights/rsub24_v3.pt"
        class_names = [
            "Bin",
            "Bin blue",
            "Bin red",
            "Red spiral",
            "Coral",
            "Nautilus",
            "Path Marker",
            "Red buoy",
            "Blue spiral",
            "Tube Worm",
        ]
        class_colors = [
            (100, 50, 70),
            (200, 50, 60),
            (70, 180, 20),
            (0, 0, 255),
            (255, 255, 0),
            (150, 255, 150),
            (150, 255, 0),
            (255, 0, 255),
            (255, 0, 0),
            (150, 0, 0),
        ]
        camera = rospy.get_param("~image_topic", "/camera/down/image_color")
        self.vs = VisionStack(
            layers=[
                ResizeLayer(960, 608),
                # UnderWaterImageEnhancementLayer(),
                # CustomLayer("pool_effect", make_underwater_effect),
                ObjectDetectionLayer(
                    path_to_weights,
                    0.7,
                    0.5,
                    class_names,
                    class_colors,
                    True,
                ),
            ],
        )
        self.image_sub = Image_Subscriber(camera, self.detection_callback)

    def detection_callback(self, msg):
        self.vs.run(msg, True)


if __name__ == "__main__":
    rospy.init_node("yolo_detections")
    YoloVision()
    rospy.spin()
