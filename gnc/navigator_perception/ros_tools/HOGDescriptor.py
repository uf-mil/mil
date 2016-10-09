# !/usr/bin/python
import cv2
import numpy as np
from sklearn.svm import SVC
import os
import sys
import argparse
import csv
import pickle

# open the roi file


class HOG(object):

    def __init__(self):
        self.clf = None

    def __getstate__(self):
        return (self.clf.kernel, self.clf.gamma, self.clf.support_vectors_, self.clf.dual_coef_)

    def train(self, in_folder):
        images_t = sorted(os.listdir(in_folder))
        images = []
        for i in images_t:
            if i.endswith('.png'):
                images.append(i)

        roi_file = open(in_folder + "/roi.csv", 'rb')
        roi_file = csv.reader(roi_file, delimiter=' ', quotechar='|')

        hog = cv2.HOGDescriptor((8, 8), (8, 8), (4, 4), (8, 8), 9)
        desc_list = None
        class_list = np.array(0)

        # loop through the training images
        count = 0
        max_len = 0
        for roi_arr in roi_file:
            image = in_folder + "/" + images[count]
            # get the area of the image
            img = cv2.imread(image)
            img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            # mask = np.zeros(img.shape[:2], np.uint8)
            print int(roi_arr[0]), int(roi_arr[1]), int(roi_arr[2]), int(roi_arr[3])
            pos = img[int(roi_arr[1]):int(roi_arr[3]), int(roi_arr[0]):int(roi_arr[2])]
            # obscure the buoy in the image
            neg = img.copy()
            val = 0
            for i in range(0, img.shape[0]):
                for j in range(0, img.shape[1]):
                    if(j > int(roi_arr[2]) and j < int(roi_arr[3])):
                        neg[i][j] = val
                    else:
                        val = neg[i][j]
            print "1"

            print pos.shape

            myhog = hog.compute(pos)
            myinvhog = hog.compute(neg)

            print "3"

            len_myhog = len(myhog)
            len_myinvhog = len(myinvhog)

            myhog = np.reshape(myhog, [len_myhog / 9, 9])
            myinvhog = np.reshape(myinvhog, [len_myinvhog / 9, 9])

            print "2"

            # add hogs to an array
            if(count == 0):
                desc_list = np.vstack((myhog, myinvhog))
            else:
                desc_list = np.vstack((desc_list, myhog, myinvhog))

            if(len(desc_list) > max_len):
                max_len = len(desc_list)

            # add classifiers to array
            class_temp = np.empty(len_myhog / 9)
            class_temp.fill(1)
            class_list = np.append(class_list, class_temp)
            class_temp = np.empty(len_myinvhog / 9)
            class_temp.fill(-1)
            class_list = np.append(class_list, class_temp)
            count += 1

        # # hack
        class_list = np.delete(class_list, [0])

        print desc_list.shape
        print class_list.shape

        # train the svm
        self.clf = SVC()
        self.clf.fit(desc_list, class_list)


if __name__ == '__main__':
    usage = ("""Pass in:
             1. the folder than contains your ROI images using the manual_grabcut.py
             found in gnc > sub8_perception > sub8_vision_tools > labelling
             2. The pickle file to output to""")
    desc = "This is used for creating a pickle file for an svm trained on HOG descriptors"
    parser = argparse.ArgumentParser(usage=usage, description=desc)
    parser.add_argument('input', help="The input roi folder")
    parser.add_argument('output', help="The output pickle file")

    args = parser.parse_args(sys.argv[1:])

    hog = HOG()
    hog.train(args.input)
    pickle.dump(hog, open(args.output, "wb"))
