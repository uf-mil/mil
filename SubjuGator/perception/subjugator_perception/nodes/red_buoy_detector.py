import cv2 as cv
from mil_ros_tools import Image_Publisher, Image_Subscriber, rosmsg_to_numpy


class Red_Buoy_Detector:
    def __init__(self):
        print("Initilalized Red Buoy finder!")
