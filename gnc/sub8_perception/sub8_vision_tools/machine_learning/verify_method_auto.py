#!/usr/bin/python
import cv2
import numpy as np
import os
import argparse
import sys
from time import time
from boost_auto import gen_data, observe
import tqdm


def load_images(path, images_to_use=2):
    assert type(images_to_use) is int
    images = os.listdir(path)
    data = []

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
            if count <= 10 - images_to_use:
                continue

            if mask.max() > 1:
                mask /= mask.max()

            data.append([image, mask, images[i]])
    
            if count == 10:
                # Reset when we get to 10
                count = 0

        except KeyboardInterrupt:
            return None

        except:
            print "There was an issue with loading an image. Skipping it..."

    return data


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument(dest='impath',
                        help="The path to the images.")
    parser.add_argument(dest='classpath',
                        help="The path to the classifications.")

    print "Loading images..."
    args = parser.parse_args(sys.argv[1:])
    data = load_images(args.impath)

    report_data = {
        "Type":[],
        "Average accuracy":[],
        "Average false positives":[],
        "Min accuracy":[],
        "Max false positives":[],
        "Average execution time":[]
    }

    print "Verifying..."
    count = 0
    param_gen = gen_data()
    for m,t,d,n in param_gen:

        f_name = "{}_{}tree_{}depth.dic".format(n, t, d)
        print " ===================== "
        print f_name

        try:
            clf = cv2.Boost()
            clf.load(args.classpath + f_name)
        except:
            print "Failed to load."
            continue

        attributes = {
            'times': [],
            'true_positives': [],
            'false_positives': [],
        }
        for u_image, u_mask, image_name in data:
            if u_image is None or u_mask is None:
                continue

            print "Image:",image_name

            image = u_image[::2, ::2, :]
            mask = u_mask[::2, ::2]
            tic_observation = time()
            some_observations = observe(image)
            print '-------------------------'
            print 'Observing took {} seconds'.format(time() - tic_observation)
            tic_prediction = time()

            try:
                segmentation = [x for x in [clf.predict(obs) for obs in some_observations]]
            except:
                print "Failed to load. File probably doesn't exist"
                break

            print 'Predicting took {} seconds'.format(time() - tic_prediction)
            total_time = time() - tic_observation

            segmentation_image = np.reshape(segmentation, image[:, :, 2].shape)
            # cv2.imshow('im2', image)
            #cv2.imshow('im', (segmentation_image * 250).astype(np.uint8))

            bool_targets = mask == 1
            bool_predictions = segmentation_image == 1

            if np.sum(bool_targets) == 0:
                print "No data in frame. Skipping."
                continue

            true_positives = np.sum(bool_targets & bool_predictions) / np.sum(bool_targets).astype(np.float32)
            false_positives = np.sum(
                np.logical_not(bool_targets) & bool_predictions
            ) / np.sum(np.logical_not(bool_targets)).astype(np.float32)

            print '\tPercent correct: {}'.format(true_positives)
            print '\tFalse Positives: {}'.format(false_positives)


            attributes['true_positives'].append(true_positives)
            attributes['false_positives'].append(false_positives)
            attributes['times'].append(total_time)

            #key = chr(cv2.waitKey(100) & 0xff)
            #if key == 'q':
            #    break
            print

        try:
            print 'Average accuracy: {}'.format(np.average(attributes['true_positives']))
            print 'Average false positives: {}'.format(np.average(attributes['false_positives']))
            print 'Min accuracy: {}'.format(np.min(attributes['true_positives']))
            print 'Max false positives: {}'.format(np.max(attributes['false_positives']))
            print 'Average execution time: {}'.format(np.average(attributes['times']))

            report_data["Type"].append(f_name)
            report_data["Average accuracy"].append(np.average(attributes['true_positives']))
            report_data["Average false positives"].append(np.average(attributes['false_positives']))
            report_data["Min accuracy"].append(np.min(attributes['true_positives']))
            report_data["Max false positives"].append(np.max(attributes['false_positives']))
            report_data["Average execution time"].append(np.average(attributes['times']))
        except:
            print "Issue"
    
    # Give the results.
    print 
    print
    print "===== RESULTS ====="
    print

    for i,t in enumerate(report_data["Type"]):
        print t
        print "Average accuracy", report_data["Average accuracy"][i]
        print "Average false positives", report_data["Average false positives"][i]
        print "Min accuracy", report_data["Min accuracy"][i]
        print "Max false positives", report_data["Max false positives"][i]
        print "Average execution time", report_data["Average execution time"][i]
        print
        print "================="
        print