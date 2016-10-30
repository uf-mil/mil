#!/usr/bin/env python
import sys
from collections import deque
import cv2
import os
import numpy as np
from debug import Debug
import operator


class ScanTheCode:

    def __init__(self):

        self.image_cache = deque()
        self.last_cam_info = None
        self.mission_complete = False
        self.colors = []
        self.scan_the_code = None
        self.on = False
        self.img1 = cv2.imread('help.png', 1)
        print "made"

    def get_rectangle(self, img, bounding_rect):
        debug = Debug(wait=False)

        def make_vec(x):
            return (x[0] / np.linalg.norm(x), x[1] / np.linalg.norm(x))

        def toint(x):
            if isinstance(x, (list, tuple)):
                return map(lambda x: int(round(x)), x)
            return int(round(x))

        def in_range(pic, x, y):
            h, w = pic.shape
            if x < 0 or x > w:
                return False
            if y < 0 or y > h:
                return False
            return True

        def line_intersection(p1, vec1, p2, vec2):
            x1, y1 = p1
            x2, y2 = vec1[0] * 5 + x1, vec1[1] * 5 + y1
            x11, y11 = p2
            x22, y22 = vec2[0] * 5 + x11, vec2[1] * 5 + y11
            m1 = (y2 - y1) / (x2 - x1 + 1E-100)
            m2 = (y22 - y11) / (x22 - x11 + 1E-100)
            if abs(m1) < 1E-50:
                return x22, y2
            if abs(m2) < 1E-50:
                return x2, y22

            x = (y1 - y11 + m2 * x11 - m1 * x1) / (m2 - m1)
            y = m1 * (x - x1) + y1
            return x, y

        def get_vertical_lines(edges, img):
            # $$$$$$$$$$$$$$$$$$$$ DEBUG - REMOVE IMG PARAM $$$$$$$$$$$$$$$$$$$$$$
            e = img.copy()
            img1 = img.copy()
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
            lines = cv2.HoughLinesP(edges, 1, np.pi / 360, threshold=30, minLineLength=10)
            if lines is None:
                print "No lines at all"
                return False, None

            vert_lines = []
            for x1, y1, x2, y2 in lines[0]:
                slope = (y2 - y1) / (x2 - x1 + 1E-100)
                if abs(slope) > .8:
                    xoi, yoi = x1, y1
                    count = 0
                    # Cleaning up the line estimate
                    while in_range(edges, xoi, yoi) and count < 15:
                        if not in_range(edges, xoi - 5, yoi - 1) or not in_range(edges, xoi + 5, yoi - 1):
                            break
                        r = 3
                        check = edges[yoi - 1: yoi, xoi - r: xoi + r].flatten()
                        check = map(lambda x: abs(x), check)
                        index, value = max(enumerate(check), key=operator.itemgetter(1))
                        if value < 255:
                            break
                        xoi = xoi - r + index
                        yoi -= r
                        count += 1
                        # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                        cv2.circle(img1, (xoi, yoi), 1, (255, 0, 0), -1)
                        debug.add_image(img1, "sup", wait=33)
                        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

                    top_y = min(y2, yoi)
                    if top_y == y2:
                        vert_lines.append((y2, yoi - y2, xoi - x2))
                    if top_y == yoi:
                        vert_lines.append((yoi, y2 - yoi, x2 - xoi))
                    # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                    cv2.line(e, (xoi, yoi), (x2, y2), (0, 0, 255), 2)
            debug.add_image(e, "linees")
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
            if len(vert_lines) < 4:
                return False, None
            return True, vert_lines

        def get_loi(starting_height):
            line_count = 0
            loi_x = []
            for i in range(0, w, 1):
                if edges[starting_height, i] > 0:
                    # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                    cv2.circle(edges_1, (i, starting_height), 1, (255, 0, 0), -1)
                    debug.add_image(edges_1, "horiz", wait=0)
                    # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

                    # Check and see if there is a line there
                    hits = 0
                    for j in range(-6, 6, 3):
                        a, b = map(lambda vec: toint(vec * j), vec)
                        # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
                        cv2.circle(edges_2, (a + i, b + starting_height), 1, (255, 0, 0), -1)
                        debug.add_image(edges_2, "horiz_points")
                        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
                        if in_range(edges, a + i - 1, b + starting_height) and in_range(edges, a + i + 1, b + starting_height):
                            buff = edges[b + starting_height, a + i - 1:a + i + 1]
                        else:
                            continue
                        if sum(buff) > 0:
                            hits += 1
                            print "Yay! you got a line! Hits = {}".format(hits)
                        else:
                            print "No line :( Hits = {}".format(hits)
                    if hits >= 2:
                        line_count += 1
                        cv2.waitKey(0)
                        if line_count == 2 or line_count == 3:
                            loi_x.append(i)
            return line_count, loi_x
        # preprocess the image, make sure that you have good naming conventions
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        gray = cv2.bilateralFilter(gray, 11, 13, 13)
        xmin, ymin, xmax, ymax = bounding_rect
        roi = gray[ymin:ymax, xmin:xmax]
        edges = cv2.Canny(roi, 50, 150, apertureSize=3)

        # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
        edges_1 = edges.copy()
        edges_2 = edges.copy()
        edges_3 = edges.copy()
        edges_4 = edges.copy()
        edges_5 = edges.copy()
        # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

        # get the verticle lines
        succ, lines = get_vertical_lines(edges, roi)
        if not succ:
            print "No Vertical lines found"
            return False, None

        line = max(lines, key=lambda x: np.linalg.norm([x[1], x[2]]))
        min_y, vec_y, vec_x = line
        height = (min_y + vec_y) / 2
        dir_vec = make_vec((vec_x, vec_y))

        # intersect a line through this average and see if you can get a line around that center point with the same slope.
        # Once you reach 4 lines stop and pick the two middle lines (If you don't get four return false)
        h, w = roi.shape
        hoi = toint(height)
        vec = dir_vec
        perp_vec = (-vec[1], vec[0])

        line_count, loi_x = get_loi(hoi)

        # Do this twice to accound for random missing parts of the edges
        if line_count < 4:
            print "Four lines did not show up in this picture, but {} did".format(line_count)
            line_count, loi_x = get_loi(hoi + 10)

        if line_count != 4:
            print "Four lines did not show up in this picture, but {} did".format(line_count)
            return False, None
        # Then go to the middle of these two lines, intersect a verticle line, and find the first line that intersects
        mid = (loi_x[0] + loi_x[1]) / 2
        xoi, yoi = vec[0] + mid, vec[1] + hoi
        scale = 1
        loi_y = []
        while in_range(edges, xoi, yoi):
            xoi, yoi = toint((vec[0] * scale + mid, vec[1] + scale + hoi))
            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            cv2.circle(edges_3, (xoi, yoi), 1, (255, 0, 0), -1)
            debug.add_image(edges_3, "vert_points")
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$
            if edges[yoi, xoi] > 0:
                scale = 1
                loi_y.append(yoi)
            if len(loi_y) == 2:
                break
            if len(loi_y) == 1:
                scale -= 1
            else:
                scale += 1
        if len(loi_y) != 2:
            print "Weird... We could not find any horizontal lines"
            return False, None

        # Find where the four lines intersect, get your points.
        pnts = []
        for i in range(0, 2):
            vert_p1 = (loi_x[i], height)
            horiz_p1 = (mid, loi_y[0])
            horiz_p2 = (mid, loi_y[1])

            # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
            d_vert_p1 = toint((vec[0] * 100 + vert_p1[0], vec[1] * 100 + vert_p1[1]))
            d_vert_p2 = toint((vec[0] * -100 + vert_p1[0], vec[1] * -100 + vert_p1[1]))
            d_horiz_p1 = toint((perp_vec[0] * 100 + horiz_p1[0], perp_vec[1] * 100 + horiz_p1[1]))
            d_horiz_p2 = toint((perp_vec[0] * -100 + horiz_p1[0], perp_vec[1] * -100 + horiz_p1[1]))
            d_horiz1_p1 = toint((perp_vec[0] * 100 + horiz_p2[0], perp_vec[1] * 100 + horiz_p2[1]))
            d_horiz1_p2 = toint((perp_vec[0] * -100 + horiz_p2[0], perp_vec[1] * -100 + horiz_p2[1]))
            cv2.line(edges_4, tuple(d_vert_p1), tuple(d_vert_p2), (255, 255, 255))
            cv2.line(edges_4, tuple(d_horiz_p1), tuple(d_horiz_p2), (255, 255, 255))
            cv2.line(edges_4, tuple(d_horiz1_p1), tuple(d_horiz1_p2), (255, 255, 255))
            debug.add_image(edges_4, "lines")
            # $$$$$$$$$$$$$$$$$$$$ ENDDEBUG $$$$$$$$$$$$$$$$$$$$$$

            x1, y1 = line_intersection(vert_p1, vec, horiz_p1, perp_vec)
            x2, y2 = line_intersection(vert_p1, vec, horiz_p2, perp_vec)
            pnts.append((x1, y1))
            pnts.append((x2, y2))

        # $$$$$$$$$$$$$$$$$$$$ DEBUG $$$$$$$$$$$$$$$$$$$$$$
        for p in pnts:
            cv2.circle(edges_5, (int(p[0]), int(p[1])), 3, (255, 0, 0), -1)
        debug.add_image(edges_5, "points", wait=0)
        # $$$$$$$$$$$$$$$$$$$$ END $$$$$$$$$$$$$$$$$$$$$$

        # Return true and your points
        return True, pnts

    def start_mission(self):
        while not self.mission_complete:
            cv_image = self.bridge.imgmsg_to_cv2(self.image_cache[len(self.image_cache) - 1], "bgr8").copy()
            img_clone = cv_image.copy()
            img_clone1 = cv_image.copy()

            scan_the_code = self.get_scan_the_code()

            points_2d = self.get_2d_points(scan_the_code.points, scan_the_code.header, img_clone1)
            xmin, ymin, xmax, ymax = self.get_bounding_rect(points_2d, cv_image)
            xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)

            # DEBUG
            for i, val in enumerate(points_2d):
                cv2.circle(img_clone, (int(val[0]), int(val[1])), 3, (255, 255, 255), -1)
            cv2.rectangle(img_clone, (xmin, ymin), (xmax, ymax), (255, 255, 255))
            cv2.imshow("bounding_rect", img_clone)
            cv2.waitKey(33)

            h, w, r = cv_image.shape

            self.mission_complete = self.model_tracker.track_models(cv_image)

            if(self.mission_complete):
                print "MISSION COMPLETE"

            rect = self.get_rectangle(cv_image, (xmin, ymin, xmax, ymax))

            if(rect is not None):
                self.model_tracker.register_model(rect, cv_image)

            if(rect is not None):
                for i, val in enumerate(rect):
                    cv2.circle(img_clone1, tuple(val[0]), 3, (255, 255, 255), -1)
                cv2.imshow("model", img_clone1)
                cv2.waitKey(33)

        # TODO Get the closest image in the deque

    def get_bounding_rect(self, points_2d, img):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, point in enumerate(points_2d):
            if(point[0] < xmin):
                xmin = point[0]
            if(point[0] > xmax):
                xmax = point[0]
            if(point[1] > ymax):
                ymax = point[1]
            if(point[1] < ymin):
                ymin = point[1]
        return xmin, ymin, xmax, ymax

    def get_non_furthest_points(self, i, points):
        dist_points = []
        for ind, val in enumerate(points):
            if(ind != i):
                diff = np.subtract(points[i], val)
                dist_points.append([np.linalg.norm(diff), points[ind]])

        dist_points = sorted(dist_points, key=lambda x: x[0])

        return dist_points[0][1], dist_points[1][1]


def main(args):
    s = ScanTheCode()
    img = s.img1
    h, w = img.shape[0], img.shape[1]
    s.get_rectangle(s.img1, (0, 0, w, h))


if __name__ == '__main__':
    main(sys.argv)
