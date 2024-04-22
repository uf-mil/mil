import cv2 as cv

# from mil_ros_tools import Image_Publisher, Image_Subscriber, rosmsg_to_numpy


# class Red_Buoy_Detector:
#     def __init__(self, path):
#         self.image = cv.resize(cv.imread(path), (960, 600))

#     def detect(self):


import cv2 as cv
import numpy as np

img = cv.imread("./test_images/buoys.png")
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

# blur and get edges from image with no hue
blur = cv.GaussianBlur(no_hue, (5, 5), 0)
edges = cv.Canny(blur, 70, 225)

# detect circles, since there is only one buoy, we set minDist to 500 in order to prevent false circles
circles = cv.HoughCircles(edges, cv.HOUGH_GRADIENT, 1, 500, param1=100, param2=18, minRadius=5, maxRadius=500)

# draw circles on result
if circles is not None:
    circles = np.round(circles[0, :]).astype("int")

    for (x, y, r) in circles:
        cv.circle(img, (x, y), r, (0, 255, 0), 4)

cv.imshow('No hue', no_hue)
cv.imshow('Edges', edges)
cv.imshow('Result', img)
cv.waitKey(0)
