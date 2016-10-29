import sys
import cv2
import numpy as np
import time


class Model:

    def __init__(self, prev_points, prev_frame, _id):
        self.prev_points = prev_points
        self.prev_frame = prev_frame
        self.time_started = time.time()
        self.turned_black = False
        self.colors = []
        self.prev_color = None
        self.colors_found = 0
        self.my_id = _id
        self.offset = None
        self.offset_sum = [0, 0, 0, 0]
        self.num_offset = 0

    def get_bounding_rect(self):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, cont in enumerate(self.prev_points):
            for j, _val in enumerate(cont):
                if(_val[0] < xmin):
                    xmin = _val[0]
                if(_val[0] > xmax):
                    xmax = _val[0]
                if(_val[1] < ymin):
                    ymin = _val[1]
                if(_val[1] > ymax):
                    ymax = _val[1]
                # print xmin, ymin, xmax, ymax

        return xmin, ymin, xmax, ymax

    def get_color(self, scalar):
        blackness1 = scalar[0] - scalar[1]
        blackness2 = scalar[1] - scalar[2]
        if(abs(blackness1) < 30 and abs(blackness2) < 30):
            o = [0, scalar[0] - scalar[1], scalar[0] - scalar[2], 0]
            self.num_offset += 1
            self.offset_sum = np.add(o, self.offset_sum)
            self.offset = np.append(self.offset_sum / self.num_offset, 0)
            return 'k'

        if(self.offset is not None):
            scalar = np.add(scalar, self.offset)

        a = scalar[0]
        b = scalar[1]
        c = scalar[2]
        if(abs(a - b) < 10 and (b - c) < 70 and b > c and a > c):
            return 'y'
        elif(a > b and a > c):
            return 'b'
        elif(b > a and b > c):
            return 'g'
        elif(c > a and c > b):
            return 'r'
        else:
            return 'g'

    def check_for_colors(self):
        if(self.colors_found == 3):
            return True, self.colors
        xmin, ymin, xmax, ymax = self.get_bounding_rect()
        scalar = cv2.mean(self.prev_frame[ymin:ymax, xmin:xmax])
        cv2.imshow("colros", self.prev_frame[ymin:ymax, xmin:xmax])
        cv2.waitKey(33)
        color = self.get_color(scalar)
        print "ID", self.my_id
        print "color", color

        if(self.prev_color is not None):
            changed = False

            if(color != self.prev_color):
                changed = True

            if(color == 'k'):
                print "turned black"
                self.turned_black = True
                self.colors_found = 0
                del self.colors[:]

            elif (self.turned_black and changed):
                self.colors_found += 1
                self.colors.append(color)
                print "changed"
                print color

        print "cols", self.colors
        self.prev_color = color
        print "---"
        return False, []
