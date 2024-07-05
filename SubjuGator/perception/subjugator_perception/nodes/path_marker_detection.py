#!/usr/bin/env python3
import cv2
import numpy as np
import rospy
from mil_ros_tools import (
    Image_Subscriber,
)
from vision_stack import (
    BinThresholdingLayer,
    CustomLayer,
    GaussianLayer,
    HoughTransformLayer,
    ResizeLayer,
    VisionStack,
)

__author__ = "Daniel Parra"


class PathMarkerDetection:
    def __init__(self):
        camera = rospy.get_param("~image_topic", "/camera/down/image_color")

        def color_magnification(img, color_tuple):
            if len(img.shape) == 2:  # Grayscale image (height, width)
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)

            height, width, channels = img.shape

            color_image = np.ones((height, width, channels), dtype=np.uint8) * np.array(
                color_tuple,
                dtype=np.uint8,
            )

            absolute_error = cv2.subtract(img, color_image)

            all_white_image = np.ones((height, width, 3), dtype=np.uint8) * 255

            all_white_image = all_white_image[:height, :width, :]

            similarity_image = np.square(
                all_white_image.astype(np.float32) - absolute_error,
            )

            similarity_image = (
                (similarity_image - np.min(similarity_image))
                / (np.max(similarity_image) - np.min(similarity_image))
                * 255
            )

            similarity_image = similarity_image.astype(np.uint8)

            return (similarity_image, None)

        def turn_to_grayscale(img, *args):
            if len(img.shape) == 2:  # Grayscale image (height, width)
                img = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
            img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            return (cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR), None)

        self.vs = VisionStack(
            layers=[
                ResizeLayer(960, 608),
                GaussianLayer((21, 21), 50),
                CustomLayer("orange_magnification", color_magnification, (52, 60, 71)),
                # RGBMagnificationLayer('R'),
                CustomLayer("custom_grayscale", turn_to_grayscale),
                # GrayscaleLayer(),
                BinThresholdingLayer(220, 255),
                HoughTransformLayer(
                    threshold=100,
                    min_line_length=20,
                    max_line_gap=0,
                    pass_post_processing_img=True,
                ),
            ],
        )

        self.image_sub = Image_Subscriber(camera, self.detection_callback)

    def detection_callback(self, msg):
        # Create Image from array
        self.vs.run(msg, True)


if __name__ == "__main__":
    rospy.init_node("vision_pipeline_test")
    PathMarkerDetection()
    rospy.spin()
