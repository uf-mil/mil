#!/usr/bin/env python

import numpy as np
import cv2

img = cv2.imread("/home/zobelisk/Pictures/pipe3.png")

#obtain one image
hsv_img = cv2.cvtColor(img, cv2.COLOR_RGB2HSV)
_,width,height = hsv_img.shape[::-1]

#mask for only blue (to get only the water for the most part)
lower = np.array([0, 0, 0], dtype="uint8")
upper = np.array([50, 50,50], dtype="uint8")
mask = cv2.inRange(hsv_img, lower, upper)

white = False
black = False
vertical_lines = []
start = 0
stop = 0

for i in range(width):
    if mask[(height/2), i] != 0 and not white:

        white = True
        black = False
        vertical_lines.append(i)
        start = i

    elif mask[(height/2), i] == 0 and not black:

        white = False
        black = True
        stop = i

if len(vertical_lines) == 1:
    center_pixel = vertical_lines[0]
else:
    #There is more than on vertical line
    center_pixel = width/2

print(center_pixel)
print(stop - start)

info = [center_pixel, stop-start]

'''
base_of_dock = None
left_wall = None
right_wall = None

#find base of dock starting from the bottom center of image to ensure we are facing correct side of dock
for i in range(height):
    if mask[(height-i-1), (width/2)] == 0:
        print("we found the base of the dock")
        base_of_dock = height-i-1
        break

#if we hit something not blue before reaching halfway through the image, we are on the wrong side
if base_of_dock > height/2:
    print("we are on wrong side")
    defer.returnValue(None)

#find left wall of docking area
for i in range(width/2):
    
    #look to left of pixel until we hit black
    if mask[(base_of_dock + 20), (width/2 - i)] == 0:
        left_wall = width/2 - i
        print("Left pixel of docking area is: ", left_wall)
        break

    if i == width/2 - 1:
        print("there is no left side of docking area")
        defer.returnValue(None)

#find right wall of docking area
for i in range(width/2):
    
    #look to right of pixel until we hit black
    if mask[(base_of_dock + 20), (width/2 + i)] == 0 and i != 0:
        right_wall = width/2 + i
        print("Right pixel of docking area is: ", right_wall)
        break

    if i == width/2 - 1:
        print("there is no right side of docking area")
        defer.returnValue(None)

midpoint = (left_wall + right_wall) / 2

#helpful for debugging to ensure we truly found the center of teh docking area
for i in range(height):
    mask[i, midpoint] = 0

#send out debugging image
mask_msg = self.bridge.cv2_to_imgmsg(mask, "mono8")
self.image_debug_pub.publish(mask_msg)

#return that we are on the correct side and the difference in pixels between
#midpoint of docking area and center of image
print("This is the pixel diff: ", width/2 - midpoint)
defer.returnValue(width/2 - midpoint)
'''

cv2.imshow("pic",mask)
cv2.waitKey(0) 
cv2.destroyAllWindows()