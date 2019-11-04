# !/usr/bin/python
import cv2
import numpy as np
from sklearn.svm import SVC
import os

# open the roi file

base = os.path.dirname(os.path.abspath(__file__))
folder = base + "/train_vid"
f = open(base + "/roi", 'r')

hog = cv2.HOGDescriptor((64, 64), (64, 64), (4, 4), (64, 64), 9)
desc_list = None
class_list = np.array(0)

# loop through the training images
count = 0
for i in os.listdir(os.path.abspath(folder)):
    # get the roi from the file
    path = folder + "/" + i
    roi_str = f.readline()
    roi_str = roi_str.replace("\n", "")
    roi_arr = roi_str.split(", ")
    # get the area of the image
    img = cv2.imread(path)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    mask = np.zeros(img.shape[:2], np.uint8)
    pos = img[int(roi_arr[0]):int(roi_arr[1]), int(roi_arr[2]):int(roi_arr[3])]
    # obscure the buoy in the image
    neg = img.copy()
    val = 0
    for i in range(0, img.shape[0]):
        for j in range(0, img.shape[1]):
            if(j > int(roi_arr[2]) and j < int(roi_arr[3])):
                neg[i][j] = val
            else:
                val = neg[i][j]

    # cv2.imshow("hey", pos)
    # cv2.waitKey(0)
    # cv2.imshow("hey", neg)
    # cv2.waitKey(0)
    # random neg
    # start = 70
    # neg = img[start:start + 128, 0:128]

    # cv2.imshow("hey", neg)
    # cv2.waitKey(0)

    # get the hog descriptors
    myhog = hog.compute(pos)
    myinvhog = hog.compute(neg)

    len_myhog = len(myhog)
    len_myinvhog = len(myinvhog)

    myhog = np.reshape(myhog, [len_myhog / 9, 9])
    myinvhog = np.reshape(myinvhog, [len_myinvhog / 9, 9])

    # add hogs to an array
    if(count == 0):
        desc_list = np.vstack((myhog, myinvhog))
    else:
        desc_list = np.vstack((desc_list, myhog, myinvhog))

    # add classifiers to array
    class_temp = np.empty(len_myhog / 9)
    class_temp.fill(1)
    class_list = np.append(class_list, class_temp)
    class_temp = np.empty(len_myinvhog / 9)
    class_temp.fill(-1)
    class_list = np.append(class_list, class_temp)
    count += 1

# hack
class_list = np.delete(class_list, [0])

# train the svm
clf = SVC()
clf.kernel = 'linear'
clf.loss = 'hinge'
clf.fit(desc_list, class_list)

coef_file = open(base + "/coef", 'w')
for i in range(0, len(clf.coef_[0])):
    if(i == len(clf.coef_[0]) - 1):
        coef_file.write(str(clf.coef_[0][i]))
    else:
        coef_file.write(str(clf.coef_[0][i]) + "\n")

print "Done"
