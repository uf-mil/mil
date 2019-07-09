"""Model for the ScanTheCode that tracks its own color."""
import cv2
from mil_misc_tools.text_effects import fprint
import numpy as np
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

    def check_for_colors(self, image, colors, debug):
        draw = image.copy()
        hsv = cv2.cvtColor(colors, cv2.COLOR_BGR2HSV)
        red_vote = 0
        black_vote = 0
        yellow_vote = 0
        green_vote = 0
        blue_vote = 0
        v_sum = 0
        h_sum = 0
        s_sum = 0
        tot = 0
        for c in hsv:
            hue, sat, val = c
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
        avgh, avgs, avgv = np.round(h_sum / tot), np.round(s_sum / tot), np.round(v_sum / tot)
        fprint("Average H {}".format(avgh), msg_color="green")
        fprint("Average S {}".format(avgs), msg_color="green")
        fprint("Average V {}".format(avgv), msg_color="green")
        color = 'n'
        if blue_vote == m:
            color = 'b'
        if red_vote == m:
            color = 'r'
        if yellow_vote == m:
            color = 'y'
        if green_vote == m:
            color = 'g'
        if black_vote == m:
            color = 'k'
        cv2.putText(draw, color, (20, 20), 1, 2, (255, 0, 0))
        debug.add_image(draw, "adlj", topic="colors")
        return color
