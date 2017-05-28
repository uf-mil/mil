#!/usr/bin/env python
from __future__ import division
import numpy as np
import cv2
import argparse
import time

NAME = 'Colorspaces'
clicked = True
total = 0
iterations = 0


def viz_colorspaces(rgb_image, viz_scale, disp_size=None):

    '''
    Returns image that shows the channels of the BGR, HSV, and LAB colorspace
    representations of a given image
    '''

    global clicked
    global total
    global iterations

    # Create grid of reduced size image panels
    w, h = (int(rgb_image.shape[1] * viz_scale), int(rgb_image.shape[0] * viz_scale)) if disp_size is None \
        else (int(disp_size[0] / 4), int(disp_size[1] / 3))
    viz = np.zeros((h * 3, w * 4, 3), dtype=np.uint8)
    labels = [['B', 'G', 'R'], ['H', 'S', 'V'], ['L', 'A', 'B']]

    def flatten_channels(img, labels):
        ''' Return horizontal stacking of single channels with a label overlayed '''
        assert len(labels) == img.shape[2]
        flat = np.hstack(cv2.split(img))
        for i, label in enumerate(labels):
            cv2.putText(flat, label, (i * w, int(h * 0.15)), cv2.FONT_HERSHEY_PLAIN, viz_scale * 5, 255, 2)
        return flat

    # Display small version of original image to the left of the grid
    rgb_small = cv2.resize(rgb_image, dsize=(w, h))
    viz[h:h * 2, :w] = rgb_small

    # Create small versions of image converted to HSV and LAB colorspaces
    images_and_labels = \
        zip([rgb_small, cv2.cvtColor(rgb_small, cv2.COLOR_BGR2HSV), cv2.cvtColor(rgb_small, cv2.COLOR_BGR2LAB)],
            labels)

    # Set 3x3 grid of single channel images from 3 3-channel images
    make_3deep = lambda x: np.repeat(x[:, :, np.newaxis], 3, axis=2)  # Needed for displaying in color

    t0 = time.time()
    if clicked:
        # print 'iterations', iterations
        iterations = iterations + 1

        def hist_curve(im, width, height):
            """
            Given an image in any color space (BGR, HSV, LAB)
            Return three lists for each channel in that color space.
            Each of the three return lists contains a list with two elements
            Those two elements are :
            - The image of the histogram in the requested shape (width, height)
            - The indices where the image of the histogram is non_zero
            """

            y = []
            # https://github.com/opencv/opencv/blob/master/samples/python/hist.py
            # lines 32 - 39 (with slight modifications)
            for ch in xrange(3):
                original_image = np.copy(im)
                bins = np.arange(255).reshape(255, 1)
                hist_item = cv2.calcHist([original_image], [ch], None, [255], [0, 255])
                cv2.normalize(hist_item, hist_item, 0, height, cv2.NORM_MINMAX)
                hist = np.int32(np.around(hist_item))
                pts = np.int32(np.column_stack((bins, hist)))
                pts[:, 0] = pts[:, 0]*(width/255)
                cv2.polylines(original_image, [pts], False, (1, 1, 255))
                y.append(np.flipud(original_image))
            return y

        hists_and_indices = []
        for image in images_and_labels:
            hists_and_indices.append(hist_curve(image[0], w, h))

        images_and_labels = zip(hists_and_indices, labels)

        # Space represents each colorspace
        # 0 : BGR
        # 1 : HSV
        # 2 : LAB
        space = 0
        for i, hists in enumerate(hists_and_indices):
            # Each 'hists' variable is represent a set of histograms for a given color space
            viz[h*space:h*(space + 1), w:, 1] = flatten_channels(hists[0], labels[i])
            viz[h*space:h*(space + 1), w:, 2] = flatten_channels(hists[0], labels[i])
            viz[h*space:h*(space + 1), w:, 0] = flatten_channels(hists[0], labels[i])
            space = space + 1
        t1 = time.time()
        total = total + (t1 - t0)
        if iterations % 5 == 0:
            print 'total: ', total
            print 'time: ', total/iterations
    else:
        viz[:, w:] = np.vstack(map(make_3deep, map(lambda x: flatten_channels(*x), images_and_labels)))
    return viz

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--use_cam', help='Use webcam (number)', type=int)
    parser.add_argument('-v', '--use_video', help='Use video', type=str)
    parser.add_argument('-r', '--use_ros_topic', help='Use ROS topic', type=str)
    parser.add_argument('-s', '--scale', help='Scale for image processing', type=float, default=0.5)
    args = parser.parse_args()

    if args.use_cam is not None or args.use_video:
        if args.use_cam is not None:
            cap = cv2.VideoCapture(args.use_cam)
        else:
            cap = cv2.VideoCapture(args.use_video)

        while(True):
            success, rgb_orig = cap.read()

            if not success:
                break

            cv2.imshow(NAME, viz_colorspaces(rgb_orig, args.scale, disp_size=(500, 500)))
            cv2.waitKey(5)

        cap.release()

    elif args.use_ros_topic is not None:
        key = None

        def disp_image(img):
            cv2.imshow(NAME, viz_colorspaces(img, args.scale))
            cv2.waitKey(10)

            def set_clicked(event, x, y, flags, param):

                global clicked
                if event == cv2.EVENT_LBUTTONDOWN:
                    clicked = not clicked
            cv2.setMouseCallback(NAME, set_clicked)

        import rospy
        import mil_ros_tools
        rospy.init_node(NAME)
        mil_ros_tools.Image_Subscriber(topic=args.use_ros_topic, callback=disp_image)
        rospy.spin()

    cv2.destroyAllWindows()
