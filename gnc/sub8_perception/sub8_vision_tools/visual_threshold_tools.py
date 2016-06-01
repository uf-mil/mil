#!/usr/bin/python

from __future__ import division
import traits
import traitsui
from mayavi import mlab
import cv2
import numpy as np
import vtk

output = vtk.vtkFileOutputWindow()
# Throw the stupid vtk runtime errors into a burning pit
output.SetFileName("/dev/null")
vtk.vtkOutputWindow().SetInstance(output)

color_ranges = {
    'hsv': np.array([
        [0, 179],  # Stupid OpenCV saving 1 bit...
        [0, 255],
        [0, 255]
    ]),
    'bgr': np.array([
        [0, 255],
        [0, 255],
        [0, 255]
    ])
}


def np_inrange(vector, lower_range, upper_range):
    sat_lower = np.min(vector > lower_range, axis=1)
    sat_upper = np.min(vector < upper_range, axis=1)
    return np.logical_and(sat_upper, sat_lower)


def mlab_color_imshow(image, **kwargs):
    '''Imshow with color in mlab
    Adapted from [1]
        [1] http://stackoverflow.com/a/24471211/5451259
    '''
    alpha = np.ones(image.shape[0] * image.shape[1]) * 255

    image_LUT_array = np.c_[image.reshape(-1, 3), alpha]
    image_LUT_array[:, 0], image_LUT_array[:, 2] = np.copy(image_LUT_array[:, 2]), np.copy(image_LUT_array[:, 0])
    LUT_lookup = np.arange(image.shape[0] * image.shape[1]).reshape(image.shape[0], image.shape[1])

    mlab_imshow = mlab.imshow(LUT_lookup, colormap='binary', **kwargs)
    mlab_imshow.module_manager.scalar_lut_manager.lut.table = image_LUT_array
    return mlab_imshow


def points_with_labels(x, y, z, labels, downsample=1, **kwargs):
    label_range = np.unique(labels)
    maxlabel = np.max(label_range)

    for label_candidate in label_range:
        color_hsv = ((label_candidate + 1) / (maxlabel + 1), 0.7, 0.8)
        x_ = x[labels == label_candidate][::downsample]
        y_ = y[labels == label_candidate][::downsample]
        z_ = z[labels == label_candidate][::downsample]
        mlab.points3d(x_, y_, z_, color=color_hsv, colormap='hsv', **kwargs)


def denormalize(val, _min, _max):
    return (val * _max) + _min


class ExtentDialog(traits.api.HasTraits):
    """A dialog to graphically adjust the extents of a threshold range
    """

    # Data extents
    data_x_min = traits.api.Float
    data_x_max = traits.api.Float
    data_y_min = traits.api.Float
    data_y_max = traits.api.Float
    data_z_min = traits.api.Float
    data_z_max = traits.api.Float

    x_min = traits.api.Range(0.0, 1.0, 0.0)
    x_max = traits.api.Range(0.0, 1.0, 1.0)
    y_min = traits.api.Range(0.0, 1.0, 0.0)
    y_max = traits.api.Range(0.0, 1.0, 1.0)
    z_min = traits.api.Range(0.0, 1.0, 0.0)
    z_max = traits.api.Range(0.0, 1.0, 1.0)

    ranges = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0]
    ]).transpose()
    filter_box = traits.api.Instance(traits.api.HasTraits, allow_none=False)
    image = None

    @traits.api.on_trait_change('x_min, x_max, y_min, y_max, z_min, z_max')
    def update_extent(self):
        conditions = (self.x_min < self.x_max, self.y_min < self.y_max, self.z_min < self.z_max)
        if (self.filter_box is not None and all(conditions)):

            self.ranges = np.array([
                [
                    denormalize(self.x_min, self.data_x_min, self.data_x_max),
                    denormalize(self.x_max, self.data_x_min, self.data_x_max)
                ],
                [
                    denormalize(self.y_min, self.data_y_min, self.data_y_max),
                    denormalize(self.y_max, self.data_y_min, self.data_y_max)
                ],
                [
                    denormalize(self.z_min, self.data_z_min, self.data_z_max),
                    denormalize(self.z_max, self.data_z_min, self.data_z_max)
                ]
            ])
            self.filter_box.mlab_source.x = self.ranges[0, :]
            self.filter_box.mlab_source.y = self.ranges[1, :]
            self.filter_box.mlab_source.z = self.ranges[2, :]

            if self.image is not None:
                r = cv2.inRange(self.image, self.ranges[:, 0], self.ranges[:, 1])
                cv2.imshow("segmented", r)

    view = traitsui.api.View(
        'x_min', 'x_max', 'y_min', 'y_max', 'z_min', 'z_max',
        title='Edit extent', resizable=True)


def make_extent_dialog(ranges, image=None):
    pts = mlab.points3d(
        ranges[0],
        ranges[1],
        ranges[2],
        scale_factor=0.1
    )

    mlab.pipeline.outline(pts)
    mlab.axes()
    extent_dialog = ExtentDialog(
        data_x_min=ranges[0][0], data_x_max=ranges[0][1],
        data_y_min=ranges[1][0], data_y_max=ranges[1][1],
        data_z_min=ranges[2][0], data_z_max=ranges[2][1],
        filter_box=pts, image=image
    )
    extent_dialog.edit_traits()

    return extent_dialog
