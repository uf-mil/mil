#!/usr/bin/python
'''
    --> [X] Fast way to draw ROI on image
        --> [X] Save ROI of a bunch of images
        --> [X] Read all the ROI of the image
    --> PROBLEMS
        --> [X] Working with variable sized images
            --> [X] CHUNK IT
        --> [X] Get the background images a certain way
        --> [X] Make faster
        --> [] Make PR ready
            --> [X] Add a SVM File
            --> [X] Add a way to change the SVM File
            --> [X] Add a class to just pass in an image, and it will get the image out
            --> [X] Non Max
            --> [X] train better
            --> [] ROSIFY
        --> [] ROSIFY
            --> [] Have hog detector read images from a rostopic
            --> []
    --> EXTRA FEATURES
        --> [X] Pre-train


    --> DEBUGGING 1:
        --> You can't delete certain elements of a picture cleanly, because it compresses it into one array.
        --> Manually set background...
        --> Figure out a way to move the black box to the bottom

    --> DEBUGGING PROBLEM: Nothing showing up detectMultiscale
        --> [] Correctly get the pos and neg images
        --> [X] Put in line - Doesn't work with compute hog descriptors
            --> [X] Sliding window, using SVM

        --> Revert back to non line
            --> [] Sliding window

    --> DEBUGGING PROBLEM: Too many things are showing up
        --> Get the proper background
        --> Non Maximal suppresion
        --> normalizing the hog




'''
import cv2
import numpy as np
import os


class HOGDetector:

    def __init__(self):
        # HOGDescriptor(Size win_size=Size(64, 128), Size block_size=Size(16, 16),
        #               Size block_stride=Size(8, 8), Size cell_size=Size(8, 8),
        #               int nbins=9, double win_sigma=DEFAULT_WIN_SIGMA,
        #               double threshold_L2hys=0.2, bool gamma_correction=true,
        #               int nlevels=DEFAULT_NLEVELS);
        self.hog = cv2.HOGDescriptor((64, 64), (64, 64), (4, 4), (64, 64), 9)
        self.base = os.path.dirname(os.path.abspath(__file__))
        coef = open(self.base + "/coef", 'r')
        l = coef.read()
        arr = l.split('\n')
        coef = []
        for a in arr:
            coef.append(float(a))
        print coef
        coef = np.asarray(coef)
        self.hog.setSVMDetector(coef)

    def detect(self, img):
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        (rects, weights) = self.hog.detectMultiScale(img, winStride=(4, 4),
                                                     padding=(0, 0), hitThreshold=.15, scale=1.6,
                                                     useMeanshiftGrouping=True)
        for (x, y, w, h) in rects:
            cv2.rectangle(img, (x, y), (x + w, y + h), (0, 0, 255), 2)
        return img
