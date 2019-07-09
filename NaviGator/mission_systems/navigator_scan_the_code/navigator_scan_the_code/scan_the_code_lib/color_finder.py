"""Model for the ScanTheCode that tracks its own color."""
from __future__ import division
import cv2
from mil_misc_tools.text_effects import fprint
import numpy as np
___author___ = "Tess Bianchi"


class ColorFinder:
    """Class the represents Scan The Code Model."""

    def __init__(self):
        """Initialize ScanTheCodeModel."""
        self.colors = []
        self.black_count = 0
        self.prev_color = None
        self.curr_count = 0
        self.offset = None
        self.offset_sum = [0, 0, 0, 0]
        self.num_offset = 0

        self.color_map = {'r': np.radians(0), 'y': np.radians(60),
                          'g': np.radians(120), 'b': np.radians(240)}

    def _get_closest_color(self, hue_angle):
        """
        Returns a pair of the most likely color and the radian error associated with that prediction
            `hue_angle` := The radian value associated with hue [0, 2*pi]
        Colors are found from `self.color_map`
        """
        c = np.cos(hue_angle)
        s = np.sin(hue_angle)
        error = np.inf
        likely_color = 'undef'
        for color, h_val in self.color_map.iteritems():
            cg = np.cos(h_val)
            sg = np.sin(h_val)
            # We need a signed error for some math later on so no absolute value
            this_error = np.arctan2(sg * c - cg * s, cg * c + sg * s)
            if np.abs(this_error) < np.abs(error):
                error = this_error
                likely_color = color

        # fprint("Likely color: {} with an hue error of {} rads.".format(likely_color, np.round(error, 3)))
        return [likely_color, error]

    def get_color_hsv(self, image, colors, debug):
        draw = image.copy()
        s = (colors.shape[0], 1, colors.shape[1])
        colors = np.reshape(colors, s)
        colors = colors.astype(np.float32)
        hsv = cv2.cvtColor(colors, cv2.COLOR_BGR2HSV)
        hsv = np.mean(hsv, axis=0).flatten()
        print hsv
        val, hue = hsv[0], hsv[2]
        color, error = self._get_closest_color(hue * 2)
        if val < 10:
            color = 'k'

        cv2.putText(draw, "{} | {}".format(color, error), (20, 20), 1, 2, (255, 0, 0))
        debug.add_image(draw, "colors", topic="colors")
        return color

    def get_color(self, colors, image, debug):
        draw = image.copy()
        s = (colors.shape[0], 1, colors.shape[1])
        colors = np.reshape(colors, s)
        colors = colors.astype(np.float32)
        scalar = np.mean(colors, axis=0).flatten()

        blackness1 = scalar[0] - scalar[1]
        blackness2 = scalar[1] - scalar[2]
        color = "None"
        if(abs(blackness1) < 30 and abs(blackness2) < 30 and scalar[0] < 100):
            o = [0, scalar[0] - scalar[1], scalar[0] - scalar[2], 0]
            self.num_offset += 1
            self.offset_sum = np.add(o, self.offset_sum)
            color = 'k'

        if(self.offset is not None):
            scalar = np.add(scalar, self.offset)

        a = scalar[0]
        b = scalar[1]
        c = scalar[2]
        if color == 'k':
            cv2.putText(draw, "{} | {} | {} | {}".format(color, np.round(
                a), np.round(b), np.round(c)), (20, 20), 1, 2, (255, 0, 0))
            debug.add_image(draw, "colors", topic="colors")
            return color
        if(abs(a - 255) < 10 and abs(b - 255) < 10 and abs(c - 255) < 100):
            color = 'y'
        elif(a > b and a > c and abs(a - b) > 30):
            color = 'b'
        elif(b > a and b > c):
            color = 'g'
        elif(c > a and c > b):
            color = 'r'
        else:
            color = 'g'

        cv2.putText(draw, "{} | {} | {} | {}".format(color, np.round(
            a), np.round(b), np.round(c)), (20, 20), 1, 2, (255, 0, 0))
        debug.add_image(draw, "colors", topic="colors")
        return color

    def check_for_colors(self, image, colors, debug):
        color = self.get_color(colors, image, debug)
        fprint("COLOR: {}, PREV_COLOR: {}, COLORS {}".format(color, self.prev_color, self.colors), msg_color='green')
        # cv2.imshow("asfd", image)
        # cv2.waitKey(0)
        if self.prev_color is None:
            self.prev_color = color
            return False, None

        print 'count', self.curr_count
        if self.prev_color == color:
            self.curr_count += 1

        elif self.prev_color != color and self.prev_color != 'k':
            if self.curr_count > 1 and (len(self.colors) == 0 or self.colors[len(self.colors) - 1] != self.prev_color):
                self.colors.append(self.prev_color)

            self.curr_count = 0

        fprint("LENGTH {}".format(len(self.colors)), msg_color="red")

        if len(self.colors) == 3:
            fprint("YOU DID IT! {}".format(self.colors), msg_color='yellow')
            return True, self.colors

        if color == 'k':
            self.black_count += 1

        if self.black_count == 3:
            fprint("THREE BLACKS FOUND... CLEARING", msg_color="red")
            self.black_count = 0
            self.prev_color = color
            self.colors = []
            return False, None

        fprint(self.colors, msg_color='green')
        self.prev_color = color
        return False, None
