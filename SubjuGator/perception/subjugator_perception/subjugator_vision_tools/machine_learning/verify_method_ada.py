import argparse
import pickle
import sys
from time import time

import cv2
import numpy as np
from adaboost import observe

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(dest="pkl", help="The pkl file to test on")
    parser.add_argument(dest="algorithm", help="The classifier to use")

    args = parser.parse_args(sys.argv[1:])
    data = pickle.load(open(args.pkl, "rb"))
    clf = pickle.load(open(args.algorithm, "rb"))

    attributes = {
        "times": [],
        "true_positives": [],
        "false_positives": [],
    }
    for u_image, u_mask in data:
        image = u_image[::2, ::2, :]
        mask = u_mask[::2, ::2]
        tic_observation = time()
        some_observations = observe(image)
        print("-------------------------")
        print(f"Observing took {time() - tic_observation} seconds")
        tic_prediction = time()
        segmentation = clf.predict(
            some_observations.reshape(-1, some_observations.shape[1]),
        )
        print(f"Predicting took {time() - tic_prediction} seconds")
        total_time = time() - tic_observation

        segmentation_image = np.reshape(segmentation, image[:, :, 2].shape)
        # cv2.imshow('im2', image)
        cv2.imshow("im", (segmentation_image * 250).astype(np.uint8))

        bool_targets = mask == 1
        bool_predictions = segmentation_image == 1

        true_positives = np.sum(bool_targets & bool_predictions) / np.sum(
            bool_targets,
        ).astype(np.float32)
        false_positives = np.sum(
            np.logical_not(bool_targets) & bool_predictions,
        ) / np.sum(np.logical_not(bool_targets)).astype(np.float32)

        print(f"\tPercent correct: {true_positives}")
        print(f"\tFalse Positives: {false_positives}")
        attributes["true_positives"].append(true_positives)
        attributes["false_positives"].append(false_positives)
        attributes["times"].append(total_time)

        key = chr(cv2.waitKey(100) & 0xFF)
        if key == "q":
            break

    print("Average accuracy: {}".format(np.average(attributes["true_positives"])))
    print(
        "Average false positives: {}".format(np.average(attributes["false_positives"])),
    )
    print("Min accuracy: {}".format(np.min(attributes["true_positives"])))
    print("Max false positives: {}".format(np.max(attributes["false_positives"])))
    print("Average execution time: {}".format(np.average(attributes["times"])))
