#!/usr/bin/env python

import cv2 as cv
import numpy as np

imgs = []
imgs.append(cv.imread('../../../Pictures/front.png'))
imgs.append(cv.imread('../../../Pictures/front2.png'))
imgs.append(cv.imread('../../../Pictures/foggy_front.png'))
imgs.append(cv.imread('../../../Pictures/foggy_front2.png'))
imgs.append(cv.imread('../../../Pictures/behind2.png'))

'''
for img in imgs:

    _,width,height = img.shape[::-1]
    stencil = np.zeros(img.shape).astype(img.dtype)

    fill_color = [255, 255, 255] # any BGR color value to fill with
    mask_value = 255            # 1 channel white (can be any non-zero uint8 value)

    #create custom contour
    top_left = [width/3.0, 0]
    bottom_right = [2*width/3.0, height]
    contours = [np.array([[top_left[0],top_left[1]], [top_left[0],bottom_right[1]], [bottom_right[0],bottom_right[1]], [bottom_right[0],top_left[1]]], dtype=np.int32)]
    
    stencil  = np.zeros(img.shape[:-1]).astype(np.uint8)
    cv.fillPoly(stencil, contours, mask_value)

    sel      = stencil != mask_value # select everything that is not mask_value
    img[sel] = fill_color            # and fill it with fill_color

    cv.imshow('Contours', img)
    cv.waitKey(0)
    cv.destroyAllWindows()
'''

for img in imgs:

    _,width,height = img.shape[::-1]
    hsv_img = cv.cvtColor(img,cv.COLOR_BGR2HSV)

    lower = np.array([100, 50, 50], dtype="uint8")
    upper = np.array([125, 255,255], dtype="uint8")
    mask = cv.inRange(hsv_img, lower, upper)

    base_of_dock = None
    left_wall = None
    right_wall = None

    #find base of dock down center of image to ensure we are facing correct side of dock
    for i in range(height):
        if mask[(height-i-1), (width/2)] == 0:
            base_of_dock = height-i-1
            break
    
    ignoreme = False
    if base_of_dock > height/2:
        print("Go to other side!")
        ignoreme = True
    else:
        print("Scan for symbols")

    if not ignoreme:
        #Here we would scan for symbols and shift to left or right

        #find base of dock down center of image again
        for i in range(height):
            if mask[(height-i-1), (width/2)] == 0:
                base_of_dock = height-i-1
                break
        
        #find left wall of docking area
        for i in range(width/2):
            
            #look to left of pixel until we hit black
            if mask[(base_of_dock + 20), (width/2 - i)] == 0:
                left_wall = width/2 - i
                break

        print(left_wall)

        #find right wall of docking area
        for i in range(width/2):
            
            #look to right of pixel until we hit black
            if mask[(base_of_dock + 20), (width/2 + i)] == 0 and i != 0:
                right_wall = width/2 + i
                break

        print(right_wall)

        midpoint = (left_wall + right_wall) / 2

        for i in range(height):
            mask[i, midpoint] = 0

    cv.imshow('Contours', mask)
    cv.waitKey(0)
    cv.destroyAllWindows()

#cnts = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
#cnts = cnts[0] if len(cnts) == 2 else cnts[1]
#
#cv.drawContours(img, cnts, -1, (0, 255, 0), 3)