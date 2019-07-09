"""Finds a rectangle in the image."""
import cv2
import numpy as np
import sys
import scipy.cluster.hierarchy as hcluster
___author___ = "Tess Bianchi"


class RectangleFinder(object):
    """Class that contains functionality to find rectangles."""

    def __init__(self):
        """Initialize RectangleFinder class."""
        # PARAMS HERE
        # The smaller this is the stricter it is in detecting clusters
        self.cluster_thresh = 4
        # The larger it is the closer you must be to detect lines
        self.min_line_length = 7
        # The larger it is the stricter it is to detect lines
        self.min_percent_image = .25
        # The smaller this is the stricter the algorithm is
        self.two_line_diff = 15

    def _in_range(self, pic, x, y):
        if len(pic.shape) == 3:
            h, w, r = pic.shape
        else:
            h, w = pic.shape
        if x < 0 or x >= w:
            return False
        if y < 0 or y >= h:
            return False
        return True

    def _make_rec(self, xmin, ymin, xmax, ymax):
        return [(xmin, ymin), (xmax, ymin), (xmin, ymax), (xmax, ymax)]

    def _draw_circle(self, img, xmin, ymin, xmax, ymax, color=(255, 255, 0)):
        cv2.circle(img, (xmin, ymin), 2, color, -1)
        cv2.circle(img, (xmax, ymin), 2, color, -1)
        cv2.circle(img, (xmin, ymax), 2, color, -1)
        cv2.circle(img, (xmax, ymax), 2, color, -1)
        self.debug.add_image(img, "good", topic="points_and")

    def _get_lines(self, edges):
        h, w = edges.shape
        lines = []
        for i in range(0, w):
            column = edges[0: h, i]
            max_length = -sys.maxsize
            max_top = 0
            max_bottom = 0
            curr_count = 0
            curr_top = 0
            for j, val in enumerate(column):
                if val == 255:
                    curr_count += 1

                if val == 0:
                    if curr_count > max_length:
                        max_length = curr_count
                        max_bottom = j
                        max_top = curr_top
                    curr_top = j
                    curr_count = 0
            if max_length > self.min_line_length:
                cv2.line(self.edges_1, (i, max_top), (i, max_bottom), (255, 0, 0))
                lines.append((max_length, max_top, max_bottom, i))

        lines = np.array(lines)
        if len(lines) == 0:
            return False, None

        xs = lines[:, 3]
        xs = xs.reshape(len(xs), 1)
        if len(xs) == 0 or len(xs) == 1:
            return False, None

        thresh = self.cluster_thresh
        clusters = hcluster.fclusterdata(xs, thresh, criterion="distance")
        # cv2.putText(self.edges_1, str(max_cl), (10, 30), 1, 2, (0, 0, 255))
        self.debug.add_image(self.edges_1, "edges_circle", topic="first_lines")

        # Put all the clusters in an array
        lines_by_cluster = {}
        for i, c in enumerate(clusters):
            if c not in lines_by_cluster:
                lines_by_cluster[c] = []
            lines_by_cluster[c].append(lines[i])

        new_lines = []
        # Run heuristics to decide if they are valid lines
        for cluster in lines_by_cluster.keys():
            lines = lines_by_cluster[cluster]
            bottom_y = -sys.maxsize
            top_y = sys.maxsize
            left_x = sys.maxsize
            right_x = -sys.maxsize
            xval = 0
            for line in lines:
                length, top, bottom, x = line
                if top < top_y:
                    top_y = top
                if bottom > bottom_y:
                    bottom_y = bottom
                if x > right_x:
                    right_x = x
                if x < left_x:
                    left_x = x
                if xval == 0:
                    xval = x
            if bottom_y - top_y > h * self.min_percent_image:
                new_lines.append((bottom_y - top_y, top_y, bottom_y, xval))

        new_lines = sorted(new_lines, key=lambda x: x[3])
        return True, new_lines

    def get_rectangle(self, roi, debug=None):
        """
        Get the rectangle that has changing colors in the roi.

        Returns boolean success value and the four rectangle points in the image
        """
        self.debug = debug

        if roi is None:
            return False, None

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 13, 13)
        edges = cv2.Canny(gray, 50, 150, apertureSize=3)
        debug.add_image(edges, "edges")
        # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
        self.edges_1 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        self.edges_2 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        self.edges_3 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        self.edges_4 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        self.edges_5 = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

        h, w = edges.shape
        succ, new_lines = self._get_lines(edges)

        if not succ:
            return False, None

        for i, line in enumerate(new_lines):
            length, top, bottom, x = line
            cv2.line(self.edges_3, (x, bottom), (x, top), (0, 0, 255))
        debug.add_image(self.edges_3, "new_lines", topic="new_lines")

        if len(new_lines) != 3 and len(new_lines) != 4 and len(new_lines) != 2:
            return False, None

        if len(new_lines) == 4:
            line1 = new_lines[1]
            line2 = new_lines[2]
            midx = (line1[3] + line2[3]) / 2
            midy = (line1[2] + line1[1]) / 2
            xmin, xmax, ymin, ymax = int(midx - w * .05), int(midx + w * .05), int(midy - h * .1), int(midy + h * .1)
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
            self._draw_circle(self.edges_3, xmin, ymin, xmax, ymax)
            self.debug.add_image(self.edges_3, "debug", topic="points")
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
            return True, self._make_rec(xmin, ymin, xmax, ymax)

        if len(new_lines) == 3:
            line1 = new_lines[0]
            line2 = new_lines[1]
            line3 = new_lines[2]
            dist1 = abs(line1[3] - line2[3])
            dist2 = abs(line2[3] - line3[3])
            midy = (line1[2] + line1[1]) / 2
            midx = line2[3]
            if dist2 > dist1:
                xmin, xmax, ymin, ymax = midx, int(midx + w * .1), int(midy - h * .1), int(midy + h * .1)
                # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
                self._draw_circle(self.edges_3, xmin, ymin, xmax, ymax)
                self.debug.add_image(self.edges_3, "debug", topic="points")
                # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
                return True, self._make_rec(xmin, ymin, xmax, ymax)

            if dist1 >= dist1:
                xmin, xmax, ymin, ymax = int(midx - w * .1), midx, int(midy - h * .1), int(midy + h * .1)
                # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
                self._draw_circle(self.edges_3, xmin, ymin, xmax, ymax)
                self.debug.add_image(self.edges_3, "debug", topic="points")
                # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
                return True, self._make_rec(xmin, ymin, xmax, ymax)

        if len(new_lines) == 2:
            line1 = new_lines[0]
            line2 = new_lines[1]
            dist = abs(line1[3] - line2[3])
            if dist < w * .2:
                return False, None
            midx = (line1[3] + line2[3]) / 2
            midy = (line1[2] + line1[1]) / 2
            xmin, xmax, ymin, ymax = int(midx - w * .05), int(midx + w * .05), int(midy - h * .1), int(midy + h * .1)
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
            self._draw_circle(self.edges_3, xmin, ymin, xmax, ymax)
            self.debug.add_image(self.edges_3, "debug", topic="points")
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $s$$$$$$$$$$$$$$$$$$$$$
            return True, self._make_rec(xmin, ymin, xmax, ymax)

        return False, None
