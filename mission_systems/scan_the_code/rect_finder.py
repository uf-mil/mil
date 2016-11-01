import cv2
import numpy as np
import operator
import sys


class RectangleFinder(object):

    def __init__(self):
        pass

    def make_vec(self, x):
        return (x[0] / np.linalg.norm(x), x[1] / np.linalg.norm(x))

    def toint(self, x):
        if isinstance(x, (list, tuple)):
            return map(lambda x: int(round(x)), x)
        return int(round(x))

    def in_range(self, pic, x, y):
        h, w = pic.shape
        if x < 0 or x >= w:
            return False
        if y < 0 or y >= h:
            return False
        return True

    def get_lines(self, edges, img):
        # $$$$$$$$$$$$$$$$$$$$ DEBUG - REMOVE IMG PARAM $$$$$$$$$$$$$$$$$$$$$$
        img1 = img.copy()
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

        lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=30, minLineLength=10)
        if lines is None:
            print "No lines at all"
            return False, None, None, None

        vert_lines = []
        horiz_line = sys.maxint
        slopes = np.array([0, 0])
        vert_count = 0
        for x1, y1, x2, y2 in lines[0]:
            slope = (y2 - y1) / (x2 - x1 + 1E-100)
            # If this slope is vertical
            if abs(slope) > .8:
                vert_count += 1
                xoi, yoi = x1, y1
                top_x, top_y = x1, y1
                bottom_x, bottom_y = x1, y1
                count = 0
                slice_length = 3
                # Cleaning up the line estimate
                while self.in_range(edges, xoi - slice_length, yoi - 1)
                        and self.in_range(edges, xoi + slice_length, yoi - 1)
                        and count < 15:

                    # Get a horizontal slice one pixel up, and find where the line is at that y value
                    check = edges[yoi - 1: yoi, xoi - slice_length: xoi + slice_length].flatten()
                    index, value = max(enumerate(check), key=operator.itemgetter(1))
                    if value < 255:
                        break
                    xoi += - slice_length + index
                    yoi += - slice_length
                    top_x, top_y = xoi, yoi
                    count += 1

                    # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                    # cv2.circle(img1, (xoi, yoi), 1, (255, 0, 0), -1)
                    # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

                    slopes += np.array([bottom_y - top_y, bottom_x - top_x])

                vert_lines.append((top_x, bottom_x, top_y, bottom_y))

            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            # self.debug.add_image(img1, "sup", wait=33)
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

            # If this slope is horizontal
            if abs(slope) < .2:
                if y1 < horiz_line:
                    horiz_line = y1

            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            cv2.line(img1, (x1, y1), (x2, y2), (0, 0, 255), 2)
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

        # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
        self.debug.add_image(img1, "linees")
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

        h, w = edges.shape
        if horiz_line == sys.maxint or vert_count < 2 or horiz_line > h / 3:
            return False, None, None, None

        vert_left = min(vert_lines, key=lambda x: x[0])
        vert_right = max(vert_lines, key=lambda x: x[0])
        slope = slopes / vert_count

        return True, [vert_left, vert_right], horiz_line + 2, slope

    def get_rectangle(self, edges, roi, debug):
        self.edges = edges
        self.debug = debug
        debug.add_image(edges, "myyedges")
        edges_more = cv2.Canny(roi, 30, 50, apertureSize=3)
        debug.add_image(edges_more, "myyedges1")
        # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
        self.edges_1 = edges.copy()
        self.edges_2 = edges.copy()
        self.edges_3 = edges.copy()
        self.edges_4 = edges.copy()
        self.edges_5 = edges.copy()
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

        # get the horizontal and vetical lines in the image
        succ, v_lines, h_line, slope = self.get_lines(edges, roi)
        if not succ:
            print "Proper lines not found"
            return False, None

        xoi_left_top, xoi_left_bottom, left_ymin, left_ymax = v_lines[0]
        xoi_right_top, xoi_right_bottom, right_ymin, right_ymax = v_lines[1]
        yoi_top = h_line

        left_edge = max(xoi_left_top, xoi_left_bottom) + 4
        right_edge = min(xoi_right_top, xoi_right_bottom) - 4
        top_edge = yoi_top

        starting_y = (max(left_ymax, right_ymax) + min(left_ymin, right_ymin)) / 2
        starting_x = (xoi_left_top + xoi_right_top) / 2

        xoi = left_edge
        yoi = starting_y

        # edges_more_clone = edges_more.copy()

        while self.in_range(edges_more, xoi, yoi - 3) and self.in_range(edges_more, xoi, yoi + 3):
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            # cv2.circle(edges_more_clone, (xoi, yoi), 1, (255, 0, 0), -1)
            # self.debug.add_image(edges_more_clone, "horiz_points", wait=33)
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
            
            if np.sum(edges_more[yoi - 3:yoi + 3, xoi]) >= 255:
                left_edge = xoi
                break
            xoi += 1

        xoi = right_edge

        while self.in_range(edges_more, xoi, yoi - 3) and self.in_range(edges_more, xoi, yoi + 3):
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            # cv2.circle(edges_more_clone, (xoi, yoi), 1, (255, 0, 0), -1)
            # self.debug.add_image(edges_more_clone, "horiz_points", wait=33)
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
            if np.sum(edges_more[yoi - 3:yoi + 3, xoi]) >= 255:
                right_edge = xoi
                break
            xoi -= 1

        xoi = starting_x
        yoi = top_edge
        while self.in_range(edges_more, xoi, yoi):
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            # cv2.circle(edges_more_clone, (xoi, yoi), 1, (255, 0, 0), -1)
            # self.debug.add_image(edges_more_clone, "horiz_points", wait=33)
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
            if edges_more[yoi, xoi] == 255:
                top_edge = yoi
                break
            yoi += 1

        cv2.circle(self.edges_5, (left_edge, top_edge), 3, (255, 0, 0), -1)
        cv2.circle(self.edges_5, (right_edge, top_edge), 3, (255, 0, 0), -1)
        cv2.circle(self.edges_5, (left_edge, top_edge + 20), 3, (255, 0, 0), -1)
        cv2.circle(self.edges_5, (right_edge, top_edge + 20), 3, (255, 0, 0), -1)

        debug.add_image(self.edges_5, "pooop", wait=33)
        return True, [(left_edge, top_edge), (right_edge, top_edge), (left_edge, top_edge + 20), (right_edge, top_edge + 20)]
