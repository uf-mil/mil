#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import tf.transformations as trns

__author__ = "Kevin Allen"


class Threshold(object):
    def __init__(self, low, high, conversion_code=None, in_space='BGR', thresh_space='BGR'):
        '''
        Convience class to store lower and upper bounds for doing a color threshold in OpenCV

        See examples for clearer understanding of parameters.
        @param low A list represnting the lower acceptable values for thresholding an image
        @param high A list representing the high acceptable values for thresholding an image
        @param conversion_code When not None, passed to cv2.cvtColor to change colorspace before thresholding image
        @param in_space All caps string that's a valid OpenCV colorspace ('HSV', 'BGR', 'LAB', etc),
                        used to determine conversion_code if conversion_code is None
        @param thresh_space All caps string that's a valid OpenCV colorspace, used to convert
                            image before thresholding. Used to determine conversion_code if it's None

        ex:
        Threshold((0, 0, 0), (255, 255, 255)) # Thresholds BGR image in BGR colorpace
        Threshold([25, 180 180], np.array([50, 190, 200]), thresh_space='LAB')  # Thresholds BGR image in LAB
        Threshold((50, 50, 50), [200, 200, 200], in_space='HSV', thresh_space='LAB') # Threshold HSV image in LAB
        '''
        assert isinstance(low, (tuple, list, np.ndarray)), 'param lower must be a tuple/list/np.ndarray'
        assert isinstance(high, (tuple, list, np.ndarray)), 'param upper must be a tuple/list/np.ndarray'
        self.low = np.array(low)
        self.high = np.array(high)
        self.in_space = in_space
        self.thresh_space = thresh_space
        # If conversion code not specified, try to form it from other params
        if conversion_code is None and in_space != thresh_space:
            try:
                self.conversion_code = getattr(cv2, 'COLOR_{}2{}'.format(in_space, thresh_space))
            except AttributeError:
                raise AttributeError('Could not determine conversion code from params.\
                                 Are [{}, {}] valid OpenCV colorspaces?'.format(in_space, thresh_space))
        else:
            self.conversion_code = conversion_code

    @classmethod
    def from_dict(cls, d, in_space='BGR', thresh_space=None):
        '''
        Loads thresholds from a dictionary. See examples for valid dictionaries.

        @param in_space See __init__ for valid values.
        @param thresh_space If None, loads threshold in colorspace of first key in dictionary.
                            Otherwise, see __init__ for valid values
        Valid Examples:

        { 'HSV':
            'low': [0, 20, 50],
            'high': [255, 255, 255] }
        -> Threshold([0, 20, 50], [255, 255, 255], thresh_space='HSV')

        { 'LAB': [0, 66, 66], [255, 180, 180] }
        -> Threshold([0, 66, 66], [255, 180, 180], thresh_space='LAB')
        '''
        assert isinstance(d, dict), 'd is not a dictionary'
        if thresh_space is None:
            assert len(d) > 0, 'Dictionary is empty'
            for key in d:  # Try each key for valid threshold
                try:
                    return cls.from_dict(d, in_space=in_space, thresh_space=key)
                except AttributeError:
                    pass
            raise AttributeError('No valid colorspace found in dictionary. Are {} valid OpenCV colorspaces?'.format(
                d.keys()))
        assert thresh_space in d, '{} color space not in dictionary'.format(thresh_space)
        inner = d[thresh_space]
        if 'low' in inner and 'high' in inner:
            return cls(inner['low'], inner['high'], in_space=in_space, thresh_space=thresh_space)
        assert len(inner) == 2, 'Cannot get low and high bounds from dictionary'
        return cls(inner[0], inner[1], in_space=in_space, thresh_space=thresh_space)

    @classmethod
    def from_param(cls, param, in_space='BGR', thresh_space=None):
        '''
        Loads thresholds from a ROS param. Param must be a valid dictionary, see from_dict
        '''
        return cls.from_dict(rospy.get_param(param), in_space=in_space, thresh_space=thresh_space)

    def threshold(self, img):
        if self.conversion_code is not None:
            converted = cv2.cvtColor(img, self.conversion_code)
            return cv2.inRange(converted, self.low, self.high)
        return cv2.inRange(img, self.low, self.high)

    __call__ = threshold  # Calling this will threshold an image

    def create_trackbars(self, window=None):
        '''
        Create OpenCV GUI trackbars to adjust the threshold values live.

        @param window Name of OpenCV window to put trackbars in
        '''
        if window is None:
            window = 'threshold'
            cv2.namedWindow(window)

        def set_thresh(t, i, x):
            if t == 'low':
                self.low[i] = x
            if t == 'high':
                self.high[i] = x
        for i in range(len(self.low)):
            cv2.createTrackbar('low {}'.format(i), window, int(self.low[i]), 255,
                               lambda x, _self=self, _i=i: set_thresh('low', _i, x))
        for i in range(len(self.high)):
            cv2.createTrackbar('high {}'.format(i), window, int(self.high[i]), 255,
                               lambda x, _self=self, _i=i: set_thresh('high', _i, x))

    def __str__(self):
        if self.conversion_code is not None:
            return 'Threshold from {} to {} using conversion code {}'.format(self.low, self.high, self.conversion_code)
        return 'Threshold {} images in {} colorspace from {} to {}'.format(
            self.in_space, self.thresh_space, self.low, self.high)

    def __repr__(self):
        return str((self.low, self.high, self.conversion_code))


def auto_canny(image, sigma=0.33):
    '''
    Returns a binary image of the edges in an image.
    Uses the median of the image to pick good low and upper threshold values
    for Canny, which makes it both adaptive and require less tunning.

    @param image grayscale image to find edges in
    @param sigma 0-1 float, how aggressively to find edges in an image, where
           1 will produce the most edges but also the most noise

    Credit Adrian Rosebrock
    http://www.pyimagesearch.com/2015/04/06/zero-parameter-automatic-canny-edge-detection-with-python-and-opencv/
    '''
    m = np.median(image)
    lower = int(max(0, (1.0 - sigma) * m))
    upper = int(min(255, (1.0 + sigma) * m))
    return cv2.Canny(image, lower, upper)


def contour_centroid(contour, M=None):
    '''
    Returns the centroid of the contour.
    If you have already calcualted the moments for this contour, pass it in as
    the second paramter so it not recalculated.
    source: https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html
    '''
    M = cv2.moments(contour) if M is None else M
    return (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))


def contour_mask(contour, img_shape=None, mask=None):
    '''
    Returns an image with the mask of a filled in contour given a image shape

    Either img_shape or mask MUST not be None.
    If img_shape is set, create a new mask image with that size.
    If mask is set, zero it out then draw mask.
    '''
    if mask is None:
        mask = np.empty((img_shape[0], img_shape[1], 1), dtype=np.uint8)
    mask.fill(0)
    cv2.drawContours(mask, [contour], 0, 255, -1)
    return mask


def putText_ul(img, text, org, fontFace=cv2.FONT_HERSHEY_COMPLEX_SMALL, fontScale=1, color=255,
               thickness=2, lineType=8, bottomLeftOrigin=False):
    '''
    Puts text on image like cv2.putText but shifts it such that the origin is the upper left corner of
    where the text is placed, instead of the bottom left as cv2 does by default.
    '''
    (text_width, text_height), _ = cv2.getTextSize(text, fontFace, fontScale, thickness)
    x, y = org
    y += text_height
    cv2.putText(img, text, (x, y), fontFace, fontScale, color, thickness, lineType, bottomLeftOrigin)
    return


def points_in_image(camera, points):
    '''
    Returns Mx2 np array of image points from projecting
    given 3D points. Only points within image resolution are included in output.
    points, ignoring any outside the resolution of the camera
    @param camera_mode: PinholeCameraModel instance of camera
    @param points: Nx3 np array of 3 points in camera frame
    @return Mx2 np array of projected image points which are within camera resolution.
    '''
    N = points.shape[0]
    img_points = np.empty((N, 2))
    resolution = camera.fullResolution()
    used = 0
    for i in range(N):
        img_pt = camera.project3dToPixel(points[i, :])
        if img_pt[0] < 0 or img_pt[0] > resolution[0] or img_pt[1] < 0 or img_pt[1] > resolution[1]:
            continue
        img_points[used, :] = img_pt
        used += 1
    img_points = img_points[0:used, :]
    return img_points


def roi_enclosing_points(camera, points, border=(0, 0)):
    '''
    Gets region of interest in image which encloses the projected 3D points.
    Output is given in slice format, so user can easily slice image.
    ex:
       roi = roi_enclosing_points(camera_model, object_points)
       img_object = img[roi]
       cv2.imshow('Object', img_object)
    @param camera_model: PinholeCameraModel instance
    @param points: Nx3 np array of 3 points in camera frame
    @param border: tuple (xborder, yborder),
           extra pixels to add around region of interest
    @return region of interest tuple that can be used to slice image
            in format (slice(ymin, ymax), slice(xmin, xmax))
            or None if none of the points can be seen in the image
    '''
    img_points = points_in_image(camera, points)
    if not len(img_points):
        return None
    resolution = camera.fullResolution()
    xmin = int(np.clip(np.round(np.min(img_points[:, 0]) - border[0]), 0, resolution[0]))
    xmax = int(np.clip(np.round(np.max(img_points[:, 0]) + border[0]), 0, resolution[0]))
    ymin = int(np.clip(np.round(np.min(img_points[:, 1]) - border[1]), 0, resolution[1]))
    ymax = int(np.clip(np.round(np.max(img_points[:, 1]) + border[1]), 0, resolution[1]))
    return (slice(ymin, ymax), slice(xmin, xmax))


def rect_from_roi(roi):
    '''
    Return rectangle style tuple from a roi slice tuple, like the one from roi_enclosing_points
    @param roi: region of interest slice tuple in form (slice(ymin, ymax), slice(xmin, xmax))
    @return: rectangle tuple
    ex:
        roi = roi_enclosing_points(camera_model, object_points)
        rectangle = tuple_from_slice(roi)
        cv2.rectangle(img, rectangle[0], rectangle[1], (0, 255, 0), 3)
    '''
    return ((roi[1].start, roi[0].start), (roi[1].stop, roi[0].stop))


def quaternion_from_rvec(rvec):
    '''
    Converts a rotation vector (like from cv2.SolvePnP) into a quaternion
    '''
    mat = np.eye(4)
    mat[:3, :3] = cv2.Rodrigues(rvec)[0]
    return trns.quaternion_from_matrix(mat)
