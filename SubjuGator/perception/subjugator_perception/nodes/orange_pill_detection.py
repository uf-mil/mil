#!/usr/bin/env python3
import datetime

import cv2
import numpy as np
import rospy
from image_geometry import PinholeCameraModel
from mil_ros_tools import Image_Publisher, Image_Subscriber
from std_msgs.msg import Float32


class CannyOrangePillDetection:
    THRESHOLD = 120
    ERROR = 0.05
    BRIGHTNESS_THRESHOLD = 200
    BRIGHT_OFFSET = 45

    def __init__(self):
        # Use camera data
        camera = rospy.get_param("~image_topic", "/camera/down/image_color")
        self.image_sub = Image_Subscriber(camera, self.vectorize_image)
        self.image_pub = Image_Publisher("~vector_viz_topic")
        self.image_pub_pre = Image_Publisher("~shark_viz")
        self.camera_info = self.image_sub.wait_for_camera_info()

        assert self.camera_info is not None
        self.angle_pub = rospy.Publisher("angle_offset", Float32, queue_size=10)
        self.cam = PinholeCameraModel()
        self.cam.fromCameraInfo(self.camera_info)

    def binarize_frame_by_red_channel(self, frame, threshold=100):
        """
        Binarize the frame based on the values of the red channel.
        """
        red_channel = frame[:, :, 0]
        binary_mask = (red_channel >= threshold).astype(np.uint8)
        binary_mask = binary_mask * 255
        return binary_mask

    def binarize_frame_by_thresholding(self, frame):
        gaus_image = cv2.GaussianBlur(frame, (3, 3), 5)
        blue, green, red = cv2.split(gaus_image)
        mask1 = (
            (blue - self.BRIGHT_OFFSET < red) & (blue - self.BRIGHT_OFFSET < green)
        ).astype(np.uint8) * 255
        kernel = np.ones((11, 11), np.uint8)
        mask1 = cv2.morphologyEx(mask1, cv2.MORPH_CLOSE, kernel)
        return mask1

    def preprocess_frame(self, frame):
        """
        Extracts the red channel from the frame and magnifies its values to uncover path markers.
        """
        original_frame = frame
        inverted_image = cv2.bitwise_not(original_frame)

        original_array = np.array(original_frame, dtype=np.int64)
        inv_original_array = np.array(inverted_image, dtype=np.int64)

        red_channel = original_array[:, :, 2]
        result_image = original_array.copy()

        # MIN MAX NORM
        result_image[:, :, 0] = (
            (result_image[:, :, 0] - np.min(result_image[:, :, 0]))
            / (np.max(result_image[:, :, 0]) - np.min(result_image[:, :, 0]))
            * 255
        )

        red_channel = result_image[:, :, 2]

        result_image[:, :, 0] = red_channel * (
            red_channel + inv_original_array[:, :, 0]
        )
        result_image[:, :, 1] = 0
        result_image[:, :, 2] = 0

        # MIN MAX NORM
        result_image[:, :, 0] = (
            (result_image[:, :, 0] - np.min(result_image[:, :, 0]))
            / (np.max(result_image[:, :, 0]) - np.min(result_image[:, :, 0]))
            * 255
        )

        result_image = np.clip(result_image, 0, 255)
        result_image = result_image.astype(np.uint8)

        self.image_pub_pre.publish(np.array(result_image))

        return result_image

    def extract_edges(self, bin_img):
        edges = cv2.Canny(bin_img, 20, 200)
        contours, _ = cv2.findContours(
            edges,
            cv2.RETR_EXTERNAL,
            cv2.CHAIN_APPROX_SIMPLE,
        )
        contour_image = np.zeros_like(edges)
        contour_image_bgr = cv2.cvtColor(contour_image, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(contour_image, contours, -1, (255), 2)
        lines = cv2.HoughLinesP(
            contour_image,
            1,
            np.pi / 180,
            threshold=50,
            minLineLength=100,
            maxLineGap=20,
        )
        return lines, contour_image_bgr

    def group_and_draw_vectors(self, lines, contour_image_bgr):
        radians = {}

        if lines is None:
            bgr_image = contour_image_bgr
            self.image_pub.publish(np.array(bgr_image))
            return radians

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

        return sorted_dict

    def vectorize_image(self, msg):
        # Create Image from array
        datetime.datetime.now()
        img = msg.astype("uint8")

        # Calculate the average brightness
        grayscale_image = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        average_brightness = cv2.mean(grayscale_image)[0]

        bin_frame = None

        if average_brightness > self.BRIGHTNESS_THRESHOLD:
            bin_frame = self.binarize_frame_by_thresholding(frame=img)

        else:
            preprocessed_frame = self.preprocess_frame(frame=img)

            bin_frame = self.binarize_frame_by_red_channel(
                preprocessed_frame,
                self.THRESHOLD,
            )

        lines, contour_image_bgr = self.extract_edges(bin_img=bin_frame)

        sorted_dict = self.group_and_draw_vectors(
            lines=lines,
            contour_image_bgr=contour_image_bgr,
        )

        # Display vectors on image
        if not sorted_dict:
            msg = Float32()
            msg.data = -9.99
            self.angle_pub.publish(msg)
        else:
            for key, value in sorted_dict.items():
                major_angle = sum(value) / len(value)
                major_angle * 180.0 / np.pi
                major_angle = (
                    major_angle + np.pi / 2
                    if major_angle <= 0
                    else major_angle - np.pi / 2
                )
                msg = Float32()

                msg.data = major_angle
                # if major_angle + np.pi/2:
                #     msg.data = major_angle
                self.angle_pub.publish(msg)
                break

        # bgr_image = cv2.cvtColor(contour_image_bgr)
        self.image_pub.publish(np.array(contour_image_bgr))

        datetime.datetime.now()
        # print(endtime - starttime)


if __name__ == "__main__":
    rospy.init_node("orange_pill_detection")
    CannyOrangePillDetection()
    rospy.spin()
