#!/usr/bin/env python3
import datetime

import cv2
import numpy as np
import rospy
from image_geometry import PinholeCameraModel
from mil_ros_tools import (
    Image_Publisher,
    Image_Subscriber,
)


class CannyOrangePillDetection:
    ERROR = 0.01

    def __init__(self):
        camera = rospy.get_param("~image_topic", "/camera/front/right/image_rect_color")

        self.image_sub = Image_Subscriber(camera, self.vectorize_image)
        self.image_pub = Image_Publisher("~vector_viz_topic")
        self.camera_info = self.image_sub.wait_for_camera_info()
        assert self.camera_info is not None
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)

    def vectorize_image(self, msg):
        # Create Image from array
        starttime = datetime.datetime.now()
        img = msg.astype("uint8")

        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

        # Apply GaussianBlur to the image to reduce noise
        img_rgb = cv2.GaussianBlur(img_rgb, (5, 5), 0)

        # Make Mask
        mask = (img_rgb[:, :, 2] / img_rgb[:, :, 0] > 0.5) | (
            img_rgb[:, :, 2] / img_rgb[:, :, 1] > 0.5
        )

        # Set pixels where the condition is true to black
        img_rgb[mask] = [0, 0, 0]

        # Apply Canny edge detection
        edges = cv2.Canny(img_rgb, 50, 150)

        # Vector Analysis
        # Find contours in the binary image
        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )

        # Create an empty image to draw contours on
        contour_image = np.zeros_like(edges)

        contour_image_bgr = cv2.cvtColor(contour_image, cv2.COLOR_GRAY2BGR)

        # Draw contours on the empty image
        cv2.drawContours(contour_image, contours, -1, (255), 2)  # -1 draws all contours

        lines = cv2.HoughLinesP(
            contour_image,
            1,
            np.pi / 180,
            threshold=50,
            minLineLength=30,
            maxLineGap=10,
        )

        if lines is None:
            bgr_image = cv2.cvtColor(contour_image, cv2.COLOR_GRAY2BGR)
            self.image_pub.publish(np.array(bgr_image))
            return

        radians = {}

        for i in range(len(lines)):
            line = lines[i]
            x1, y1, x2, y2 = line[0]
            angle = np.arctan2((y2 - y1), (x2 - x1))
            key = round(angle, 2)

            valid_key = (
                key
                if key in radians
                else key + self.ERROR
                if key + self.ERROR in radians
                else key - self.ERROR
                if key - self.ERROR in radians
                else -999999
            )

            if valid_key != -999999:
                radian_frequency_array = radians[valid_key]
                radian_frequency_array.append(angle)
                del radians[valid_key]
                new_key = sum(radian_frequency_array) / len(radian_frequency_array)
                new_key = round(new_key, 2)
                radians[new_key] = radian_frequency_array
            else:
                radians[key] = [angle]

            cv2.line(
                contour_image_bgr,
                (x1, y1),
                (x2, y2),
                (255 * (i / len(lines)), 255 - (255 * (i / len(lines))), 0),
                2,
            )

        sorted_dict = dict(sorted(radians.items(), key=lambda item: -len(item[1])))

        # DIsplay vectors on image
        for key, value in sorted_dict.items():
            major_angle = sum(value) / len(value)
            print(major_angle)
            break

        # bgr_image = cv2.cvtColor(contour_image_bgr)
        self.image_pub.publish(np.array(contour_image_bgr))

        endtime = datetime.datetime.now()
        print(endtime - starttime)


if __name__ == "__main__":
    rospy.init_node("orange_pill_detection")
    CannyOrangePillDetection()
    rospy.spin()
