import cv2 as cv

# from mil_ros_tools import Image_Publisher, Image_Subscriber, rosmsg_to_numpy


# class Red_Buoy_Detector:
#     def __init__(self, path):
#         self.image = cv.resize(cv.imread(path), (960, 600))

#     def detect(self):


import cv2 as cv
import numpy as np

img = cv.imread("./test_images/underwater-test.png")
img = cv.resize(img, (960, 600))
mean = cv.mean(img)[0:3]
average = np.full_like(img, (mean))
opposite = 255 - average
img_hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
opposite_hsv = cv.cvtColor(opposite, cv.COLOR_BGR2HSV)
img_hsv[:,:,0] = opposite_hsv[:,:,0]
img_hsv[:,:,1] = opposite_hsv[:,:,1]
img2 = cv.cvtColor(img_hsv, cv.COLOR_HSV2BGR)
no_hue = cv.addWeighted(img2, 0.5, img, 0.5, 0)
cv.imshow('Result', no_hue)

cv.waitKey(0)


# blur = cv.GaussianBlur(img, (5, 5), 0)
# edges = cv.Canny(blur, 70, 225)

# hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)


# gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
# gray_blurred = cv.blur(gray, (3, 3))
# value = hsv[:, :, 2]

# circles = cv.HoughCircles(gray_blurred, cv.HOUGH_GRADIENT,
#                           1, 20, param1 = 50, param2 = 30,
#                           minRadius = 1, maxRadius = 40)

# if circles is not None:
#     circles = np.round(circles[0, :]).astype("int")

#     for (x, y, r) in circles:
#         cv.circle(img, (x, y), r, (0, 255, 0), 4)

# cv.imshow('Balls', img)
# cv.imshow('Gray ball', gray_blurred)

# cv.imshow('Blurred', blur)
# cv.imshow('Edges', edges)
# cv.imshow('Grayscale HSV', value)
# cv.waitKey(0)
