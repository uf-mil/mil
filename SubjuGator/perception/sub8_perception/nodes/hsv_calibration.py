#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from mil_ros_tools import Image_Publisher, Image_Subscriber
from sub8_perception.cfg import HSVCalibrationConfig


class HSVCalibration:
    def __init__(self):
        self.camera = "/camera/front/left/image_raw"
        self.image_sub = Image_Subscriber(self.camera, self.image_cb)
        self.image_pub = Image_Publisher("/image/hsv")
        self.reconfigure_server = DynamicReconfigureServer(
            HSVCalibrationConfig, self.reconfigure
        )
        self.lower = np.array([0, 0, 0], dtype=np.uint8)
        self.upper = np.array([179, 255, 255], dtype=np.uint8)

    def image_cb(self, img):
        img = np.array(img)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, self.lower, self.upper)
        img = cv2.bitwise_and(img, img, mask=mask)
        self.image_pub.publish(img)

    @staticmethod
    def parse_string(threshes):
        ret = [float(thresh.strip()) for thresh in threshes.split(",")]
        if len(ret) != 3:
            raise ValueError("not 3")
        return ret

    def reconfigure(self, config, level):
        try:
            self.lower = np.array(self.parse_string(config["dyn_lower"]))
            rospy.logwarn("HSV lower bound below minimum value") if (
                self.lower < 0
            ).any() else None
            self.upper = np.array(self.parse_string(config["dyn_upper"]))
            rospy.logwarn("HSV upper bound above maximum values") if (
                self.upper[0] > 179
            ).any() or (self.upper[1:] > 255).any() else None

        except ValueError as e:
            rospy.logwarn(f"Invalid dynamic reconfigure: {e}")
            return self.last_config

        self.last_config = config
        return config


def main():
    rospy.init_node("hsv_calibration", anonymous=False)
    HSVCalibration()

    rospy.spin()


if __name__ == "__main__":
    main()
