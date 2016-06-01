import cv2
import numpy as np
import pickle
import argparse
import sys
import features
from sklearn.preprocessing import scale
from sub8_vision_tools.machine_learning import balanced_resample, desample_binary
"""
TODO
MUST:
    - Hard negative mining
    - Store/load method
    - Balanced sampling

SHOULD:
    - Cleaner kernel definition
    - Cleaner kernel exploration
    - Easier tool for generating data

CONSIDER:
    - Use sklearn preprocessing standar scaler
        - Whiten data, yo

- Package kernels alongside Boost
"""


def observe(image):
    im_gs = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    observations = np.reshape(image, (-1, 3)).astype(np.float32)

    kernels_run = features.conv_features(im_gs)

    kernel_observations = np.reshape(
        kernels_run, (-1, kernels_run.shape[2])
    ).astype(np.float32)

    all_observations = np.hstack(
        (
            observations,
            kernel_observations,
        )
    ).astype(np.float32)

    return all_observations


def train_on_pkl(pkl_data, images_to_use=None):
    observation_list = []
    label_list = []
    if images_to_use is None:
        images_to_use = len(pkl_data)

    print 'Generating training data...'
    for u_image, u_targets in pkl_data[:images_to_use]:
        image = u_image[::2, ::2, :]
        targets = u_targets[::2, ::2]
        target_labels = np.reshape(targets, (-1, 1)).astype(np.int32)
        observations = observe(image)
        observation_list.append(observations)
        label_list.append(target_labels)

    all_observations = np.vstack(observation_list)
    all_labels = np.vstack(label_list)

    classifier = train_classifier(all_observations, all_labels)
    return classifier


def train_classifier(x, y):
    n_trees = 5
    max_depth = 3
    parameters = {
        # "boost_type": cv2.BOOST_REAL,
        "boost_type": cv2.BOOST_GENTLE,
        # "boost_type": cv2.BOOST_DISCRETE,
        "weak_count": n_trees,
        "weight_trim_rate": 0,
        "max_depth": max_depth
    }

    boost = cv2.Boost()
    print 'Training...'
    boost.train(x, cv2.CV_ROW_SAMPLE, y, params=parameters)
    return boost


def main():
    usage_msg = ("Pass the path to a bag, and we'll crawl through the images in it")
    desc_msg = "A tool for making manual segmentation fun!"

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='pkl',
                        help="The pickle data file to train on")
    parser.add_argument('--output', type=str, help="Path to a file to output to (and overwrite)",
                        default='boost.cv2')

    args = parser.parse_args(sys.argv[1:])

    print 'Loading pickle...'
    data = pickle.load(open(args.pkl, "rb"))
    clf = train_on_pkl(data)
    image, targets = data[-1]

    some_observations = observe(image)
    prediction = [int(x) for x in [clf.predict(obs, returnSum=True) for obs in some_observations]]
    prediction2 = [int(x) for x in [clf.predict(obs) for obs in some_observations]]

    print 'Saving as {}...'.format(args.output)
    clf.save(args.output, 's')

    print 'Displaying...'
    prediction_image = np.reshape(prediction, targets.shape)
    prediction_image2 = np.reshape(prediction2, targets.shape)

    import matplotlib.pyplot as plt
    plt.figure(1)
    plt.imshow(prediction_image)
    plt.figure(2)
    plt.imshow(image)
    plt.figure(4)
    plt.imshow(prediction_image2[:, :, np.newaxis] * image[:, :])
    plt.show()
    # pickle.dump(clf, open("boost.p", "wb"))


if __name__ == "__main__":
    main()
