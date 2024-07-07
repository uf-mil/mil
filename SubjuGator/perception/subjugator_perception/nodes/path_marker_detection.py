#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rospy
from std_msgs.msg import Float32
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
    GrayscaleLayer,
    ColorMagnificationLayer
)

__author__ = "Daniel Parra"


class PathMarkerDetection:
    ERROR = 0.05

    def __init__(self):
        camera = rospy.get_param("~image_topic", "/camera/down/image_color")
        self.angle_pub = rospy.Publisher("angle_offset", Float32, queue_size=10)

        self.vs = VisionStack(
            layers=[
                ResizeLayer(960, 608),
                GaussianLayer((21, 21), 50),
                ColorMagnificationLayer((52, 60, 71)),
                GrayscaleLayer(),
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
        lines = self.vs.analysis_dict["houghTransform_5"]
        self.publish_angle_in_deg_from_list_of_lines(lines)
    
    def publish_angle_in_deg_from_list_of_lines(self, lines):
        # Create array of angles and lengths
        degrees = {}
        for i in range(len(lines)):
            line = lines[i]
            x1, y1, x2, y2 = line[0]
            angle = np.degrees(np.arctan2((y2 - y1), (x2 - x1)))
            key = round(angle, 2)

            valid_key = (
                key
                if key in degrees
                else key + self.ERROR
                if key + self.ERROR in degrees
                else key - self.ERROR
                if key - self.ERROR in degrees
                else -999999
            )

            if valid_key != -999999:
                degrees_frequency_array = degrees[valid_key]
                degrees_frequency_array.append(angle)
                del degrees[valid_key]
                new_key = sum(degrees_frequency_array) / len(degrees_frequency_array)
                new_key = round(new_key, 2)
                degrees[new_key] = degrees_frequency_array
            else:
                degrees[key] = [angle]
        
        sorted_dict = dict(sorted(degrees.items(), key=lambda item: -len(item[1])))

        if not sorted_dict:
            print("-999")
            msg = Float32()
            msg.data = -999
            self.angle_pub.publish(msg)
        else:
            for key, value in sorted_dict.items():
                major_angle = sum(value) / len(value)
                major_angle = (
                    major_angle + 90
                    if major_angle <= 0
                    else major_angle - 90
                )
                msg = Float32()
                print(major_angle)
                msg.data = major_angle
                self.angle_pub.publish(msg)
                break



if __name__ == "__main__":
    rospy.init_node("path_marker_detection")
    PathMarkerDetection()
    rospy.spin()
