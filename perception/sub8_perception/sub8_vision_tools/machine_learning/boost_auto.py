#!/usr/bin/python
import cv2
import numpy as np
import os
import argparse
import sys
import features
from sklearn.preprocessing import scale
from sub8_vision_tools.machine_learning import balanced_resample, desample_binary
import time
import tqdm

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

def gen_data():
    '''
    Set parameters here - which methods you want to use and trees/depth.
    The name will be the way that the method is displayed (since it's only a number)
    '''
    method = [cv2.BOOST_GENTLE]
    name = ["gentle"]
    trees = [8,9]
    depth = [3,5]

    for t in trees:
        for d in depth:
            for m,n in zip(method, name):
                yield m,t,d,n


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


def train_on_data(observation_list, label_list, split_factor=4):
    '''
    `to_train` should be the number of images to train on out of each group of 10.
    This lets us only train on some of the segmented images and then test on the rest.
    '''
    assert len(observation_list) / split_factor is int

    print "Done! Training on: {} images.".format(len(observation_list))

    all_observations = np.vstack(observation_list)
    all_labels = np.vstack(label_list)

    all_observations_split = np.vsplit(all_observations, split_factor)
    all_labels_split = np.vsplit(all_labels, split_factor)
        
    print
    print "Building classifier..."
    print
    param_gen = gen_data()
    for m,t,d,n in param_gen:
        f_name = "{}_{}tree_{}depth.dic".format(n, t, d)
        print "====================="
        print "Generating {}...".format(f_name)

        boost = cv2.Boost()
        parameters = {
            "boost_type": m,
            "weak_count": t,
            "weight_trim_rate": 0,
            "max_depth": d
        }

        s_time = time.time()
        process_round = 0
        # Split this into multiple passes in an attempt to free RAM (no idea if this works).
        for x, y in zip(all_observations_split ,all_labels_split):
            print "Training subset {}/{}.".format(process_round + 1, split_factor)
            boost.train(x, cv2.CV_ROW_SAMPLE, y, params=parameters)
            process_round += 1

        print "Time to complete: {}".format(time.time() - s_time)
        print "Done! Saving..."
        boost.save(f_name, 's')
        print
        

def load_images(path, images_to_use=8):
    assert type(images_to_use) is int
    images = os.listdir(path)
    observation_list = []
    label_list = []

    print "Images found in folder: {}".format(len(images) / 2 - 1)
    count = 0
    for i in tqdm.trange(len(images) / 2 - 1, desc="Loading images", unit=' images'):
        try:
            if '_mask.png' in images[i]:
                # This is the mask image - there should be a matching non mask image.
                image_name = images[images.index(images[i].replace('_mask', ''))]

                image = cv2.imread(path + image_name)
                mask = cv2.imread(path + images[i], 0)

                images.remove(image_name)
            else:
                # This is the real image - there should be a matching mask image.
                mask_name = images[images.index(images[i].replace('.png', '_mask.png'))]

                image = cv2.imread(path + images[i])
                mask = cv2.imread(path + mask_name, 0)

                images.remove(mask_name)

            count += 1
            if count > images_to_use:
                if count == 10:
                    # Reset when we get to 10
                    count = 0
                continue

            if mask.max() > 1:
                mask /= mask.max()

            target_labels = np.reshape(mask, (-1, 1)).astype(np.int32)
            observations = observe(image)

            observation_list.append(observations)
            label_list.append(target_labels)
        
        except KeyboardInterrupt:
            return None

        except:
            print "There was an issue with loading an image. Skipping it..."

    return observation_list, label_list

def main():
    usage_msg = ("Pass the path to a bag, and we'll crawl through the images in it")
    desc_msg = "A tool for making manual segmentation fun!"

    parser = argparse.ArgumentParser(usage=usage_msg, description=desc_msg)
    parser.add_argument(dest='folder',
                        help="The folder to the images to train on")
    parser.add_argument('--output', type=str, help="Path to a file to output to (and overwrite)",
                        default='boost.cv2')

    args = parser.parse_args(sys.argv[1:])

    observation_list, label_list = load_images(args.folder)
    clf = train_on_data(observation_list, label_list)

if __name__ == "__main__":
    main()
