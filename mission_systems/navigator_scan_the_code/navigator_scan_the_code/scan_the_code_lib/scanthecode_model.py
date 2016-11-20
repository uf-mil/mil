"""Model for the ScanTheCode that tracks its own color."""
import cv2
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
        self.BLACK_THRESH_V = 50
        self.BLACK_THRESH_S = 60
        self.RED_THRESH_LOW1 = 0
        self.RED_THRESH_LOW2 = 160
        self.RED_THRESH_HIGH1 = 10
        self.RED_THRESH_HIGH2 = 179
        self.BLUE_THRESH_LOW = 95
        self.BLUE_THRESH_HIGH = 135
        self.GREEN_THRESH_LOW = 41
        self.GREEN_THRESH_HIGH = 90
        self.YELLOW_THRESH_LOW = 20
        self.YELLOW_THRESH_HIGH = 40

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

    def _get_color(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        red_vote = 0
        black_vote = 0
        yellow_vote = 0
        green_vote = 0
        blue_vote = 0
        v_sum = 0
        h_sum = 0
        s_sum = 0
        tot = 0
        for i in range(0, len(roi)):
            for j in range(0, len(roi[0])):
                hue = hsv[i][j][0]
                sat = hsv[i][j][1]
                val = hsv[i][j][2]
                h_sum += hue
                v_sum += val
                s_sum += sat
                tot += 1
                if val < self.BLACK_THRESH_V or sat < self.BLACK_THRESH_S:
                    black_vote += 1
                elif hue < self.RED_THRESH_HIGH1 and hue > self.RED_THRESH_LOW1:
                    red_vote += 1
                elif hue < self.RED_THRESH_HIGH2 and hue > self.RED_THRESH_LOW2:
                    red_vote += 1
                elif hue < self.YELLOW_THRESH_HIGH and hue > self.YELLOW_THRESH_LOW:
                    yellow_vote += 1
                elif hue < self.GREEN_THRESH_HIGH and hue > self.GREEN_THRESH_LOW:
                    green_vote += 1
                elif hue < self.GREEN_THRESH_HIGH and hue > self.GREEN_THRESH_LOW:
                    blue_vote += 1
        m = max(red_vote, black_vote, yellow_vote, green_vote, blue_vote)
        fprint("Average H {}".format(h_sum / tot), msg_color="green")
        fprint("Average S {}".format(s_sum / tot), msg_color="green")
        fprint("Average V {}".format(v_sum / tot), msg_color="green")
        if blue_vote == m:
            return 'b'
        if red_vote == m:
            return 'r'
        if yellow_vote == m:
            return 'y'
        if green_vote == m:
            return 'g'
        if black_vote == m:
            return 'k'

    def check_for_colors(self, debug):
        """Tell the model to update its colors."""
        if(self.colors_found == 3):
            return True, self.colors
        xmin, ymin, xmax, ymax = self._get_bounding_rect()
        color = self._get_color(self.frame[ymin:ymax, xmin:xmax])
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        fprint("color:{}".format(color), msg_color='green')
        # %%%%%%%%%%%%%%%%%%%%%%%%DEBUG
        fc = self.frame.copy()
        cv2.putText(fc, color, (20, 20), 1, 2, (255, 0, 0))
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
