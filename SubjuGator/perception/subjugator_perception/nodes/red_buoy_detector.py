import cv2 as cv
from mil_ros_tools import Image_Publisher, Image_Subscriber, rosmsg_to_numpy


class Red_Buoy_Detector:
    def __init__(self):
        print("Initilalized Red Buoy finder!")

# import cv2 as cv
# import numpy as np

# img = cv.imread("./imgs/balls1.jpg")
# img = cv.resize(img, (960, 600))

# blur = cv.GaussianBlur(img, (5, 5), 0)
# edges = cv.Canny(blur, 70, 225)

# hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)


# gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# value = hsv[:, :, 2]

# circles = cv.HoughCircles(gray, cv.HOUGH_GRADIENT, 1, 100, 35, 20, 100, 200)


# if circles is not None:
#     circles = np.round(circles[0, :]).astype("int")

#     for (x, y, r) in circles:
#         cv.circle(img, (x, y), r, (0, 255, 0), 4)

# cv.imshow('Balls', img)
# cv.imshow('Gray ball', gray)

# cv.imshow('Blurred', blur)
# cv.imshow('Edges', edges)
# cv.imshow('Grayscale HSV', value)
# cv.waitKey(0)
