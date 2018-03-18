#!/usr/bin/env python
import json
import os
import cv2

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
    def label_point_to_numpy(points):
        # TODO: return numpy points from label object
        pass

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
    def cb(label, img):
        cv2.imshow('test' , img)
        print label
        cv2.waitKey(0)

    reader.get_labeled_images(cb)

