#!/usr/bin/env python
from __future__ import division
import numpy as np
import cv2

__author__ = 'Kevin Allen'


class ImageMux(object):
    '''
    Utility to create a customizable square grid of images, with labels and borders,
    which will generate a single image from the grid at any time. Useful for
    combining several debug images into one.

    See bottom of this file for a usage example.
    '''
    def __init__(self, size=(480, 640), shape=(2, 2), labels=None, keep_ratio=True,
                 border_color=(255, 255, 255), border_thickness=1, text_color=(255, 255, 255),
                 text_font=cv2.FONT_HERSHEY_COMPLEX_SMALL, text_scale=1, text_thickness=2):
        '''
        Contruct an ImageMux grid.
        @param size Tuple (rows, cols) representing the size of the grid image, in pixels
        @param shape Tuple (rows, cols) representing the number of smaller images in the grid
        @param labels List of strings of length shape[0] * shape[1]
        @param keep_ratio If True, do not stretch image to insert into grid pane
        @param text_color Tuple (Blue, Green, Red) color of label text
        @param text_font Integer, a valid OpenCV font to use in label text
        @param text_scale Scaling factor for label text, float
        @param text_thickness Thickness of label text, int
        '''
        self.size = np.array(size, dtype=np.uint)
        self.shape = np.array(shape, dtype=np.uint)
        self.keep_ratio = keep_ratio
        self.pane_size = np.array(self.size / self.shape, dtype=np.int)
        self.border_color = border_color
        self.border_thickness = border_thickness
        self.text_color = text_color
        self.text_font = text_font
        self.text_scale = text_scale
        self.text_thickness = text_thickness

        # If labels not specified, fill a list with None's
        if labels is None:
            self.labels = [None for _ in xrange(self.size.size)]
        else:
            assert len(labels) == self.shape[0] * self.shape[1], 'not enough labels'
            self.labels = labels

        self._image = np.zeros((size[0], size[1], 3), dtype=np.uint8)

    def _index_to_tuple(self, index):
        '''
        Internal helper function, returns row, col index
        from a single index integer
        '''
        return (int(index / self.shape[1]), int(index % self.shape[1]))

    def _apply_decorations(self):
        '''
        Internal helper function, adds border lines and label text to internal image.
        '''
        # Add border if thickness > 0
        if self.border_thickness > 0:
            for row in xrange(1, self.shape[0]):  # Add horizontal line for rows 1 - m
                y = int(self.pane_size[0] * row)
                cv2.line(self._image, (0, y), (self.size[1], y), self.border_color, self.border_thickness)
            for col in xrange(1, self.shape[1]):  # Add vertical line for rows 1 - n
                x = int(self.pane_size[1] * col)
                cv2.line(self._image, (x, 0), (x, self.size[0]), self.border_color, self.border_thickness)

        # Add label text for each pane if it is not None
        for i, label in enumerate(self.labels):
            if label is None:
                continue
            tup = self._index_to_tuple(i)
            (text_width, text_height), _ = cv2.getTextSize(label, self.text_font, self.text_scale, self.text_thickness)
            x = int(self.pane_size[1] * tup[1])
            y = int(self.pane_size[0] * tup[0] + text_height)
            # Adjust text position to not overlap border
            if tup[0] != 0:
                y += self.border_thickness
            if tup[1] != 0:
                x += self.border_thickness
            cv2.putText(self._image, label, (x, y),
                        self.text_font, self.text_scale, self.text_color, self.text_thickness)

    def set_image(self, key, img):
        '''
        Sets the content of one pane in the image grid.
        @param The index of the pane to set to the data of img
               If an integer -> sets pane at index, counting left to right, then top to bottom
               If a tuple (row, col) -> set pane at specified (index 0) row and column
        @param img numpy array with shape (m, n, 3) or (m, n, 1) representing the image to insert
               in the pane specified in key. If a one channel image, first convert grayscale to BGR.
               If keep_ratio was True in contructor, will add black bars as necessary to fill pane.
               Otherwise, use standard cv2.resize to fit img into pane.
        @throw AssertionError if key is wrong type of out of bounds
        '''
        assert isinstance(img, np.ndarray), 'img must be numpy array'
        # If image is grayscale, convert to 3 channel
        if len(img.shape) == 2 or img.shape[2] == 1:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

        if isinstance(key, int):  # Accept a single index, ex 5 -> (2, 1)
            key = self._index_to_tuple(key)
        assert isinstance(key, tuple), 'must be tuple'
        assert len(key) == 2, 'index best be 2D'
        assert key[0] < self.shape[0] and key[1] < self.shape[1], 'out of bounds'
        rows = slice(key[0] * self.pane_size[0], (key[0] + 1) * self.pane_size[0])
        cols = slice(key[1] * self.pane_size[1], (key[1] + 1) * self.pane_size[1])
        if self.keep_ratio:
            row_count = rows.stop - rows.start
            col_count = cols.stop - cols.start
            ratio = np.array([img.shape[0] / row_count, img.shape[1] / col_count])
            scale = 1 / np.max(ratio)
            size = (int(img.shape[1] * scale), int(img.shape[0] * scale))
            v_border = int((row_count - size[1]) / 2)
            h_border = int((col_count - size[0]) / 2)
            rows = slice(rows.start + v_border, rows.start + size[1] + v_border)
            cols = slice(cols.start + h_border, cols.start + size[0] + h_border)
            self._image[rows, cols] = cv2.resize(img, size)
        else:
            size = (self.pane_size[1], self.pane_size[0])
            self._image[rows, cols] = cv2.resize(img, size)

    __setitem__ = set_image  # Overload index [] operator to set image

    @property
    def image(self):  # accessing .image will draw decorations onto image then return the full image grid
        self._apply_decorations()
        return self._image

    __call__ = image  # Overload () operator to access grid image


if __name__ == '__main__':
    '''
    ImageMux is intended to be used as a class, not an executable.
    The following is an example of how to use it in a python program.
    Creates a 2x2 grid of Racoon images with labels, using some custom parameters.

    To run this yourself, download some images, put them in $HOME/Pictures/[1.jpg, 2.jpg, 3.jpg, 4.jpg]
    '''
    import os
    labels = ['Chubby Racoon', 'Kiddo Racoons', 'wide', 'tall', 'big wide', 'big tall']  # Create strings for labels
    images = [cv2.imread(os.path.join(os.environ['HOME'], 'Pictures', str(i + 1) + '.jpg')) for i in xrange(2)]
    # Add strange ratio white blocks to test keep_ratio flag
    images.append(255 * np.ones((20, 201, 3), dtype=np.uint8))     # A small, wide image
    images.append(255 * np.ones((200, 20, 3), dtype=np.uint8))     # A small, tall image
    images.append(255 * np.ones((200, 2000, 3), dtype=np.uint8))   # A large, wide image
    images.append(255 * np.ones((2000, 200, 3), dtype=np.uint8))   # A large, tall image
    t = ImageMux(size=(500, 900), border_color=(0, 0, 255), border_thickness=3, shape=(3, 2),
                 labels=labels, text_scale=1, keep_ratio=True)
    for i in xrange(len(images)):
        t[i] = np.array(images[i])
    cv2.imshow('Grid', t.image)
    print 'Press any key in GUI window to exit'
    cv2.waitKey(0)
