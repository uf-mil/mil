#!/usr/bin/env python
from abc import ABCMeta, abstractmethod, abstractproperty
from mil_vision_tools import LabelBoxParser, contour_mask
import cv2
import numpy as np
from sklearn.naive_bayes import GaussianNB
import pandas

__author__ = 'Kevin Allen'


class ContourClassifier(object):
    '''
    Abstract class to represent a classifier for contours
    within images. See GaussianColorClassifier for an example.

    Concrete classes must define all methods and properties marked as abstract.
    '''
    __metaclass__ = ABCMeta  # Treat

    def __init__(self, classes):
        '''
        Constructs a ContourClassifier.
        @param classes: a list of class names (as strings), ex: ['red', 'white', 'blue']
        '''
        self.classes = classes

    @classmethod
    @abstractproperty
    def FEATURES(cls):
        '''
        For concrete classes, must return a list of strings naming the features returned by get_features
        ex: ['red_mean', 'blue_mean', 'green_mean']
        '''
        pass

    @abstractmethod
    def get_features(self, img, mask):
        '''
        For concrete classes, returns an array of numerical features based on on the image and contour mask
        '''
        pass

    @abstractmethod
    def classify_features(self, features):
        '''
        For concrete classes, classify a contour based on it's set of features returned from get_features
        @param features: a list of numerical features in the order returned by get_features
        @return: the index of the class which is the most probabable classification based on the features
        '''
        pass

    @abstractmethod
    def train(self, features, classes):
        '''
        For concrete classes, train the classifier based on the set of features and their labeled class index.
        @param features: np array with shape (n_samples, m_features), where each row is a list of features
                         in the order returned from get_features
        @param classes: list with shape (n_samples) of labeled class index corosponding to each row of features
        '''
        pass

    def score(self, features, classes):
        '''
        Returns the classification accuracy based on the set of features and labeled class indicies
        @param features: (n_samples, m_features) np array where each row is a list of features
        @param classes: (n_samples) array with labeled class indicies
        @return: A proportion accuracy correct_classificiations / len(classes)
        '''
        n = len(classes)
        correct = 0
        for i in range(n):
            classification = self.classify_features(features[i, :].reshape(1, -1))
            if classification == classes[i]:
                correct += 1
        return float(correct) / n

    def string_to_class(self, strings):
        '''
        Maps a single string or list of strings representing a class in the set of those passed to the contructor
        to the corresponding integer index representing this class, used in most functions for training / classification
        '''
        map(self.classes.index, strings)

    def class_to_string(self, classes):
        '''
        Maps a single integer or list of integers representing a class index to the corresponding
        class string in the set of those passed to the constructor.
        '''
        map(self.classes.__getitem__, classes)

    def classify(self, img, mask):
        '''
        Classify a contour
        @param img: 2d image
        @param mask: binary mask image representing the contour
        @return: the class index this is most probable given the features in that contour
        '''
        features = self.get_features(img, mask)
        return self.classify_features(features)

    def read_from_csv(self, filename):
        '''
        Read in the features and labeled classes from filename, assuming it was saved
        in the format obtained from save_csv.
        '''
        df = pandas.DataFrame.from_csv(filename)
        classes = df.values[:, 0]
        features = df.values[:, 1:]
        return features, classes

    def train_from_csv(self, filename):
        '''
        Train the classifier given the labeled feature found in filename
        which should be in the format obtained from save_csv
        '''
        features, classes = self.read_from_csv(filename)
        self.train(features, classes)

    def save_csv(self, features, classes, filename='training.csv'):
        '''
        Save the features and labeled classes to a csv file
        to be used in later training.
        '''
        features = np.array(features)
        classes = np.array(classes)
        classes = classes.reshape((classes.shape[0], 1))
        data = np.hstack((classes, features))
        df = pandas.DataFrame(data=data, columns=['Class'] + self.FEATURES)
        df.to_csv(filename)

    def extract_labels(self, labelfile, image_dir):
        '''
        Extract features and labeled classes from a project labeled on labelbox.io
        @param labelfile: the json file containing the labels for the project
        @param image_dir: directory where source images for the project can be found
        @return features, classes; np arrays that can be used in self.train or self.save_csv
        '''
        labler = LabelBoxParser(labelfile, image_dir)
        label_features = []
        label_classes = []

        def labeled_img_cb(label, img):
            for l in label['Label']:
                if l in self.classes:
                    for single_label in label['Label'][l]:
                        points = LabelBoxParser.label_to_contour(single_label, img.shape[0])
                        mask = contour_mask(points, img.shape)
                        features = self.get_features(img, mask)
                        label_features.append(features)
                        label_classes.append(self.classes.index(l))

        labler.get_labeled_images(labeled_img_cb)
        return np.array(label_features), np.array(label_classes)


class GaussianColorClassifier(ContourClassifier):
    '''
    A contour classifier which classifies a contour
    based on it's mean color in BGR, HSV, and LAB colorspaces,
    using a Gaussian classifier for these features.

    For more usage info, see class ContourClassifier
    '''
    FEATURES = ['B', 'G', 'R', 'H', 'S', 'V', 'L', 'A', 'B']

    def __init__(self, classes):
        super(GaussianColorClassifier, self).__init__(classes)
        self.classifier = GaussianNB()

    def get_features(self, img, mask):
        mean = cv2.mean(img, mask)
        mean = np.array([[mean[:3]]], dtype=np.uint8)
        mean_hsv = cv2.cvtColor(mean, cv2.COLOR_BGR2HSV)
        mean_lab = cv2.cvtColor(mean, cv2.COLOR_BGR2LAB)
        features = np.hstack((mean.flatten(), mean_hsv.flatten(), mean_lab.flatten()))
        return features

    def classify_features(self, features):
        return self.classifier.predict(features)

    def train(self, features, classes):
        self.classifier.fit(features, classes)
