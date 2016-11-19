"""Model for the ScanTheCode that tracks its own color."""
import cv2
import numpy as np
from navigator_tools import fprint
___author___ = "Tess Bianchi"


class ScanTheCodeModel:
    """Class the represents Scan The Code Model."""

    def __init__(self, points, frame):
        """Initialize ScanTheCodeModel."""
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

    def _get_bounding_rect(self):
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

    def _get_color(self, scalar):
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
        elif a < b and a < c and abs(b - c) < 40:
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
        """Tell the model to update its colors."""
        if(self.colors_found == 3):
            return True, self.colors
        xmin, ymin, xmax, ymax = self._get_bounding_rect()
        scalar = cv2.mean(self.frame[ymin:ymax, xmin:xmax])
        color = self._get_color(scalar)
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        fprint("color:{}".format(color), msg_color='green')
        fprint("scalar:{}".format(scalar), msg_color='green')
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        fc = self.frame.copy()
        cv2.putText(fc, color + str(scalar), (20, 20), 1, 2, (255, 0, 0))
        debug.add_image(fc, "adlj", topic="colors")
        if(self.prev_color is not None):

            changed = False
            if(color != self.prev_cached_color):
                changed = True

            if color != self.prev_color:
                self.count_same_colors = 0

            if (color == self.prev_color or self.prev_color == 'k') and self.turned_black and color != 'k':
                self.count_same_colors += 1

            if(color == 'k'):
                self.turned_black = True
                self.colors_found = 0
                self.count_same_colors = 0
                del self.colors[:]

            elif self.turned_black and changed and self.count_same_colors > 0:
                self.colors_found += 1
                self.colors.append(color)
                self.prev_cached_color = color
                self.count_same_colors = 0

            fprint("count_same_colors:{}".format(self.count_same_colors), msg_color='green')

        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        fprint("colors:{}".format(self.colors), msg_color='green')
        print "---"
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        self.prev_color = color
        return False, []
