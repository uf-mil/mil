#!/usr/bin/env python
import json
import os
import cv2
import numpy as np

class LabelBoxParser(object):
    '''
    Class to read an exported labelbox annotation file (must be JSON formatted)
    and call a user specified callback function with this parsed annotation along with the
    image it annotates.
    '''
    def __init__(self, labelfile, image_dir='.'):
        '''
        @param labelfile: name of json file containing labelbox annotations
        @param image_dir: directory where all images will be found (should be same place you uploaded them from).
                          It is expected that the images in this directory have the same name as when they
                          were uploaded to labelbox.
        '''
        f = open(labelfile)
        self.labels = json.load(f)
        self.image_dir = image_dir

    @staticmethod
    def label_to_contour(points, height):
        '''
        Return cv2 style contour from a label. Requires img height so the y values can be correct
        @param points: list of points from a label in format [{'x': 542.4, 'y': 5454}, {'x': 232.12, 'y': 652}, ...]
        @param height: height of the image the contour comes from. Use img.shape[0]
        ex:
        def cb(label, img):
            for key in label['Label']: # For each class
                for polygon in label['Label'][key]: # For each segmentation with that class
                    points = LabelBoxParser.label_to_contour(polygon, img.shape[0])
                    cv2.drawContours(img, [points], -1, (255, 255, 255), 3)
        '''
        nppts = np.zeros((len(points), 2))
        for i, pt in enumerate(points):
            nppts[i, 0] = pt['x']
            nppts[i, 1] = height - pt['y']
        return np.array(nppts, dtype=int)

    def get_labeled_images(self, cb):
        '''
        Runs through all images, loads them into a np array (cv2 style) and passes
        them to the specified callback function.

        cb should be in form:
            def cb(label, img):
                # Do feature extraction, etc
        '''
        for label in self.labels:
            imgname = os.path.join(self.image_dir, label['External ID'])
            img = cv2.imread(imgname)
            cb(label, img)


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Example of labelbox reader class. Displays images with label')
    parser.add_argument('labels', type=str,
                         help='JSON file with labels exported from labelbox.io')
    parser.add_argument('dir', type=str, default='.', nargs='?',
                        help='directory to find images which were uploaded to labelbox for this dataset')
    args = parser.parse_args()

    reader = LabelBoxParser(args.labels, image_dir=args.dir)

    import cv2
    from mil_vision_tools import contour_centroid, contour_mask, putText_ul
    def cb(label, img):
        for key in label['Label']:
            for polygon in label['Label'][key]:
                points = LabelBoxParser.label_to_contour(polygon, img.shape[0])
                centroid = contour_centroid(points)
                cv2.circle(img, (centroid[0], centroid[1]), 3, (255, 255, 255))
                cv2.drawContours(img, [points], -1, (255, 255, 255), 3)
                putText_ul(img, key, centroid)

        cv2.imshow('test', img)
        cv2.waitKey(0)
    reader.get_labeled_images(cb)

