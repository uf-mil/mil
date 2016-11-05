import cv2
import numpy as np


class Model:

    def __init__(self, points, frame):
        self.points = points
        self.frame = frame
        self.turned_black = False
        self.colors = []
        self.prev_color = None
        self.colors_found = 0
        self.offset = None
        self.offset_sum = [0, 0, 0, 0]
        self.num_offset = 0
        self.count_same_colors = 0
        self.prev_cached_color = None

    def get_bounding_rect(self):
        xmin = 1000
        xmax = 0
        ymin = 1000
        ymax = 0
        for i, _val in enumerate(self.points):
            if(_val[0] < xmin):
                xmin = _val[0]
            if(_val[0] > xmax):
                xmax = _val[0]
            if(_val[1] < ymin):
                ymin = _val[1]
            if(_val[1] > ymax):
                ymax = _val[1]

        return xmin, ymin, xmax, ymax

    def get_color(self, scalar):
        blackness1 = scalar[0] - scalar[1]
        blackness2 = scalar[1] - scalar[2]
        if(abs(blackness1) < 30 and abs(blackness2) < 30 and scalar[0] < 100):
            o = [0, scalar[0] - scalar[1], scalar[0] - scalar[2], 0]
            self.num_offset += 1
            self.offset_sum = np.add(o, self.offset_sum)
            return 'k'

        if(self.offset is not None):
            scalar = np.add(scalar, self.offset)

        a = scalar[0]
        b = scalar[1]
        c = scalar[2]
        if(abs(a - 255) < 10 and abs(b - 255) < 10 and abs(c - 255) < 100):
            return 'y'
        elif(a > b and a > c and abs(a - b) > 30):
            return 'b'
        elif(b > a and b > c):
            return 'g'
        elif(c > a and c > b):
            return 'r'
        else:
            return 'g'

    def check_for_colors(self, debug):
        if(self.colors_found == 3):
            return True, self.colors
        xmin, ymin, xmax, ymax = self.get_bounding_rect()
        scalar = cv2.mean(self.frame[ymin:ymax, xmin:xmax])
        color = self.get_color(scalar)
        print "color", color
        print "scalar", scalar
        fc = self.frame.copy()
        cv2.putText(fc, color + str(scalar), (20, 20), 1, 2, (255, 0, 0))
        debug.add_image(fc, "adlj", topic="colasr")
        if(self.prev_color is not None):

            changed = False

            if(color != self.prev_cached_color):
                changed = True

            if color == self.prev_color and self.turned_black and color != 'k':
                self.count_same_colors += 1

            if(color == 'k'):
                self.turned_black = True
                self.colors_found = 0
                del self.colors[:]

            elif self.turned_black and changed and self.count_same_colors == 2:
                self.colors_found += 1
                self.colors.append(color)
                self.prev_cached_color = color
                self.count_same_colors = 0

        print "cols", self.colors
        self.prev_color = color
        print "---"
        return False, []
