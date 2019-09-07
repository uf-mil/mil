#!/usr/bin/env python
from __future__ import division
import numpy as np
import cv2
import argparse

__author__ = "David Soto"

'''
Script to play a video through ROS, a webcam, or a video file
and display visual information about the colors in the video
in the RGB, HSV, and LAB colorspaces.

Usage:
rosrun mil_vision colorspaces.py -h
By pressing 'h', the GUI will display a histogram in each color
By pressing 'c', the GUI will display the precise color values at the cursor location

'''

# Globals variables which change behavior of viz_colorspaces
NAME = 'Colorspaces'
w, h = 0, 0
mouse_x, mouse_y = None, None
show_color = False
show_histogram = False
bins = np.arange(256).reshape(256, 1)


def viz_colorspaces(rgb_image, viz_scale, disp_size=None):
    '''
    Returns image that shows the channels of the BGR, HSV, and LAB colorspace
    representations of a given image
    '''

    global show_color, show_histogram
    global w, h
    global mouse_x, mouse_y

    # Pick a width and height of each image in GUI window based on start parameters
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

    # If enabled, show specific color information at current cursor location
    if show_color and mouse_x is not None and mouse_y is not None:
        # In empty bottom left panel, fill with RGB color at cursor location
        viz[h * 2:, 0:w] = rgb_small[mouse_y, mouse_x]
        t_y = 0
        for i in xrange(len(images_and_labels)):
            # Print values in each color space of current cursor location
            text = "[{l[0]}, {l[1]}, {l[2]}] = [{r[0]:>3}   {r[1]:>3}   {r[2]:>3}]".format(
                l=images_and_labels[i][1], r=images_and_labels[i][0][mouse_y, mouse_x])
            (t_w, t_h), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_PLAIN, 2.3 * viz_scale, 1)
            t_y += t_h * 2
            cv2.putText(viz, text, (0, t_y), cv2.FONT_HERSHEY_PLAIN, 2.3 * viz_scale, (255, 255, 255), 1)

    # Set 3x3 grid of single channel images from 3 3-channel images
    make_3deep = lambda x: np.repeat(x[:, :, np.newaxis], 3, axis=2)  # Needed for displaying in color
    viz[:, w:] = np.vstack(map(make_3deep, map(lambda x: flatten_channels(*x), images_and_labels)))

    # If enabled, display a color histogram in each panel
    if show_histogram:
        def get_hist_curves(img):
            ''' Return list of curves for each channel in image '''
            curves = [None for i in range(img.shape[2])]
            for ch in xrange(img.shape[2]):
                hist_item = cv2.calcHist([img], [ch], None, [256], [0, 256])
                cv2.normalize(hist_item, hist_item, 0, 255, cv2.NORM_MINMAX)
                hist = np.int32(np.around(hist_item))
                curves[ch] = np.int32(np.column_stack((bins, hist)))
            return curves

        for i in range(len(images_and_labels)):
            for c, curve in enumerate(get_hist_curves(images_and_labels[i][0])):
                mask = np.zeros(images_and_labels[i][0].shape, dtype=np.uint8)
                cv2.polylines(mask, [curve], False, (255, 0, 0))
                mask = np.flipud(mask)
                cv2.bitwise_or(viz[i * h:(i + 1) * h, (c + 1) * w:(c + 2) * w], mask,
                               dst=viz[i * h:(i + 1) * h, (c + 1) * w:(c + 2) * w])
    return viz


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('-c', '--use_cam', help='Use webcam (number)', type=int)
    parser.add_argument('-v', '--use_video', help='Use video', type=str)
    parser.add_argument('-r', '--use_ros_topic', help='Use ROS topic', type=str)
    parser.add_argument('-s', '--scale', help='Scale for image processing', type=float, default=0.5)
    args = parser.parse_args()

    print "PRESS 'h' to display histograms"
    print "PRESS 'c' to display color values"

    def image_cb(img):
        ''' Display a single image and check for mouse events '''
        cv2.imshow(NAME, viz_colorspaces(img, args.scale))
        key = cv2.waitKey(10)
        if key == ord('c'):
            global show_color
            show_color = not show_color
        if key == ord('h'):
            global show_histogram
            show_histogram = not show_histogram

        def set_clicked(event, x, y, flags, param):
            global mouse_x, mouse_y
            # Get index in any panel, not whole window
            mouse_x, mouse_y = x % w, y % h
        cv2.setMouseCallback(NAME, set_clicked)

    # Handle webcam or video file
    if args.use_cam is not None or args.use_video:
        if args.use_cam is not None:
            cap = cv2.VideoCapture(args.use_cam)
        else:
            cap = cv2.VideoCapture(args.use_video)
        while(True):
            success, rgb_orig = cap.read()
            if not success:
                break
            image_cb(rgb_orig)
        cap.release()

    # Pull images from a ROS image topic
    elif args.use_ros_topic is not None:
        import rospy
        import mil_ros_tools
        rospy.init_node(NAME, anonymous=True)
        mil_ros_tools.Image_Subscriber(topic=args.use_ros_topic, callback=image_cb)
        rospy.spin()

    else:
        print 'No video, camera, or ROS topic specified. Closing'

    cv2.destroyAllWindows()
