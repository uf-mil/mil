#!/usr/bin/env python3
import argparse
from abc import ABCMeta, abstractmethod, abstractproperty
from typing import List, Optional, Tuple, Union

import cv2
import numpy as np
import numpy.typing as npt
import pandas
from sklearn.naive_bayes import GaussianNB

from .cv_tools import contour_mask
from .labelbox_parser import LabelBoxParser

__author__ = "Kevin Allen"


def _get_param(one, two):
    """
    Helpers function to get whichever param is not None, used
    to have class defaults which can be overridden in function calls
    """
    assert one is not None or two is not None, "both params are None"
    return two if one is None else one


class ContourClassifier:
    """
    Abstract class to represent a classifier for contours within images.

    See GaussianColorClassifier for an example.

    Attributes:
        classes (List[:class:`str`]): A list of class names (as strings),
            for example: ``['red', 'white', 'blue']``.
        training_file (Optional[:class:`str`]): The name of the training file.
        labelfile (Optional[:class:`str`]): The file containing the labels.
        image_dir (Optional[:class:`str`]): The directory containing the images.
    """

    __metaclass__ = ABCMeta  # Treat

    def __init__(
        self,
        classes: List[str],
        training_file: Optional[str] = None,
        labelfile: Optional[str] = None,
        image_dir: Optional[str] = None,
    ):
        self.classes = classes
        self.training_file = training_file
        self.labelfile = labelfile
        self.image_dir = image_dir

    @classmethod
    @abstractproperty
    def FEATURES(cls) -> List[str]:
        """
        For concrete classes, must return a list of strings naming the features
        returned by get_features.

        For example, an example overload might return ``['red_mean', 'blue_mean', 'green_mean']``.
        """
        raise NotImplementedError

    @abstractmethod
    def get_features(self, img: np.ndarray, mask: np.ndarray) -> np.ndarray:
        """
        Returns an array of numerical features based on on the image and contour mask.
        This should be implemented in all child classes.

        Args:
            img (np.ndarray): The 2D image to get the features of.
            mask (np.ndarray): The contour mask to apply over the image.

        Returns:
            np.ndarray: An array of numerical features found.
        """

    @abstractmethod
    def classify_features(self, features: np.ndarray) -> int:
        """
        For concrete classes, classify a contour based on it's set of features
        returned from get_features.

        Args:
            features (np.ndarray): a list of numerical features in the order
                returned by :meth:`.get_features`.

        Returns:
            The index of the class which is the most probabable classification
            based on the features.
        """

    def feature_probabilities(self, features: np.ndarray) -> List[float]:
        """
        Allows child classes to give probabilities for each possible class given
        a features vector, instead of just one classification. By default,
        gives 1.0 to classified class and 0.0 to others.

        Returns:
            List[float]: The list of probabilities. Each member ranges from 0 to 1.
        """
        probabilities = [0.0] * len(self.classes)
        classification = self.classify_features(features)
        probabilities[classification] = 1.0
        return probabilities

    @abstractmethod
    def train(self, features: np.ndarray, classes: np.ndarray):
        """
        For concrete classes, train the classifier based on the set of features
        and their labeled class index.

        Args:
            features (np.ndarray): Array with shape ``(n_samples, m_features)``,
                where each row is a list of features in the order returned from
                :meth:`.get_features`.
            classes (np.ndarray): List with shape ``(n_samples)`` of labeled class index
                corresponding to each row of features.
        """

    def score(self, features: np.ndarray, classes: np.ndarray) -> float:
        """
        Returns the classification accuracy based on the set of features and
        labeled class indices.

        Args:
            features (np.ndarray): Array of shape ``(n_samples, m_features)``, where each row is a list of features.
            classes (np.ndarray): Array of shape ``(n_samples)`` with labeled class indices.

        Returns:
            A proportion accuracy correct_classificiations / len(classes).
        """
        n = len(classes)
        correct = 0
        for i in range(n):
            classification = self.classify_features(features[i, :].reshape(1, -1))
            if classification == classes[i]:
                correct += 1
        return float(correct) / n

    def string_to_class(self, strings: Union[List[str], str]) -> Union[List[int], int]:
        """
        Maps a single string or list of strings representing a class in the set
        of those passed to the constructor to the corresponding integer index
        representing this class, used in most functions for training / classification.

        Args:
            strings (Union[List[str], str]): The string or list of strings to return
                the index of.

        Returns:
            Union[List[int], int]: The list of indexes (if multiple strings are passed),
            or the single index if only one string is passed.
        """
        if isinstance(strings, list):
            return list(map(self.classes.index, strings))
        return self.classes.index(strings)

    def class_to_string(self, classes: Union[List[int], int]) -> Union[List[str], str]:
        """
        Maps a single integer or list of integers representing a class index to
        the corresponding class string in the set of those passed to the constructor.

        Args:
            classes (Union[List[int], int]): The list of indexes to find the associated
                classes of. Otherwise, the name of one class to find one index of.

        Returns:
            Union[List[str], str]: The list of found classes or the one found class.
        """
        if isinstance(classes, list):
            return list(map(self.classes.__getitem__, classes))
        return self.classes[classes]

    def classify(self, img: np.ndarray, mask: np.ndarray) -> int:
        """
        Classify a contour.

        Args:
            img (np.ndarray): 2D image representation.
            mask (np.ndarray): Binary mask image representing the contour.

        Returns:
            The class index this is most probable given the features in that contour.
        """
        features = self.get_features(img, mask).reshape(1, -1)
        return self.classify_features(features)

    def probabilities(self, img: np.ndarray, mask: np.ndarray) -> List[float]:
        """
        Return a vector of probabilities of the features retrieved from the contour with
        given mask corresponding to the class list.

        Args:
            img (np.ndarray): 2D image representation.
            mask (np.ndarray): Binary mask image representing the contour.

        Returns:
            List[float]: The list of probabilities across the entire image.
        """
        features = self.get_features(img, mask).reshape(1, -1)
        return self.feature_probabilities(features)

    def read_from_csv(self, training_file: Optional[str] = None):
        """
        Return features and classes from specified training file.

        Args:
            training_file (Optional[str]): The name of the training file.
        """
        training_file = _get_param(training_file, self.training_file)
        df = pandas.read_csv(training_file)
        classes = df.values[:, 1]
        features = df.values[:, 2:]
        return features, classes

    def train_from_csv(self, training_file: Optional[str] = None) -> None:
        """
        Train the classifier given the labeled feature found in filename
        which should be in the format obtained from :meth:`.save_csv`.

        Args:
            training_file (Optional[str]): The name of the training file.
        """
        training_file = _get_param(training_file, self.training_file)
        features, classes = self.read_from_csv(training_file)
        self.train(features, classes)

    def save_csv(
        self,
        features: npt.ArrayLike,
        classes: npt.ArrayLike,
        training_file: Optional[str] = None,
    ) -> None:
        """
        Save the features and labeled classes to a csv file
        to be used in later training.

        Args:
            features (npt.ArrayLike): The features found.
            classes (npt.ArrayLike): The classes used in the classifier.
            training_file (Optional[str]): The name of the file to save the data as.
        """
        training_file = _get_param(training_file, self.training_file)
        features = np.array(features)
        classes = np.array(classes)
        classes = classes.reshape((classes.shape[0], 1))
        data = np.hstack((classes, features))
        df = pandas.DataFrame(data=data, columns=["Class", *self.FEATURES])
        df.to_csv(training_file)

    def extract_labels(
        self,
        labelfile: Optional[str] = None,
        image_dir: Optional[str] = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        """
        Extract features and labeled classes from a project labeled on labelbox.io.

        Args:
            labelfile (Optional[str]): the json file containing the labels for the project
            image_dir (Optional[str]): directory where source images for the project can be found

        Raises:
            Exception: No labels were found.

        Returns:
            Tuple[np.ndarray, np.ndarray]: Tuple of the features found and classes found.
        """
        labelfile = _get_param(labelfile, self.labelfile)
        image_dir = _get_param(image_dir, self.image_dir)
        print(labelfile, image_dir)
        labler = LabelBoxParser(labelfile, image_dir)
        label_features = []
        label_classes = []

        def labeled_img_cb(label, img):
            for lab in label["Label"]:
                if lab in self.classes:
                    for single_label in label["Label"][lab]:
                        points = LabelBoxParser.label_to_contour(
                            single_label,
                            img.shape[0],
                        )
                        mask = contour_mask(points, img.shape)
                        features = self.get_features(img, mask)
                        label_features.append(features)
                        label_classes.append(self.classes.index(lab))

        labler.get_labeled_images(labeled_img_cb)
        if len(label_features) == 0:
            raise Exception("No labels found. Please check parameters")
        return np.array(label_features), np.array(label_classes)

    def main(self, params, description="Interface with a contour classifier class"):
        """
        Allows execution of the class from a command-line interface.
        """
        parser = argparse.ArgumentParser(description=description)
        parser.add_argument(
            "--training-file",
            "-t",
            type=str,
            default=None,
            help="CSV file to save/load training features and label to.",
        )
        subparser = parser.add_subparsers()
        extract = subparser.add_parser("extract", help="Extract labels")
        extract.set_defaults(cmd="extract")
        extract.add_argument(
            "--label-file",
            "-l",
            type=str,
            default=None,
            help="Path of json file containing labelbox labels",
        )
        extract.add_argument(
            "--image-dir",
            "-d",
            type=str,
            default=None,
            help="Path to directory containing images for datasets labeled by label file",
        )
        score = subparser.add_parser(
            "score",
            help="Print a accuracy score based on saved training file",
        )
        score.set_defaults(cmd="score")
        args = parser.parse_args(params)
        if args.cmd == "extract":
            features, classes = self.extract_labels(
                labelfile=args.label_file,
                image_dir=args.image_dir,
            )
            self.save_csv(features, classes, training_file=args.training_file)
        if args.cmd == "score":
            features, classes = self.read_from_csv(training_file=args.training_file)
            self.train(features, classes)
            print(f"Score: {self.score(features, classes) * 100}%")


class GaussianColorClassifier(ContourClassifier):
    """
    A contour classifier which classifies a contour
    based on it's mean color in BGR, HSV, and LAB colorspaces,
    using a Gaussian classifier for these features.

    For more usage info, see the ContourClassifier class.
    """

    FEATURES = ["B", "G", "R", "H", "S", "V", "L", "A", "B"]

    def __init__(self, classes, **kwargs):
        super().__init__(classes, **kwargs)
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

    def feature_probabilities(self, features):
        return self.classifier.predict_proba(features)

    def train(self, features, classes):
        self.classifier.fit(features, classes)
