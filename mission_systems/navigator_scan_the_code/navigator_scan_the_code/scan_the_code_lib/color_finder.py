"""Model for the ScanTheCode that tracks its own color."""
from __future__ import division
import cv2
from navigator_tools import fprint
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
        self.K_H_LOW = 0
        self.K_H_HIG = 5
        self.K_V_LOW = 0
        self.K_V_HIG = 20

        self.R_H_LOW = 0
        self.R_H_HIG = 20
        self.R_H_LOW2 = 160
        self.R_H_HIG2 = 180

        self.R_V_LOW = 230
        self.R_V_HIG = 255

        self.B_H_LOW = 100
        self.B_H_HIG = 140
        self.B_V_LOW = 100
        self.B_V_HIG = 200

        self.G_H_LOW = 41
        self.G_H_HIG = 90
        self.G_V_LOW = 100
        self.G_V_HIG = 200

        self.Y_H_LOW = 20
        self.Y_H_HIG = 40
        self.Y_V_LOW = 200
        self.Y_V_HIG = 255

    def get_color(self, image, colors, debug):
        draw = image.copy()
        s = (colors.shape[0], 1, colors.shape[1])
        colors = np.reshape(colors, s)
        colors = colors.astype(np.float32)
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
            tot += 1
            c = c.flatten()
            val, sat, hue = c[0], c[1], c[2]
            h_sum += hue
            s_sum += sat
            v_sum += val
            # if hue < self.K_H_HIG and hue > self.K_H_LOW:
            #     black_vote += 1
            if hue < self.R_H_HIG and hue > self.R_H_LOW:
                red_vote += 1
            if hue < self.R_H_HIG2 and hue > self.R_H_LOW2:
                red_vote += 1
            # elif hue < self.B_H_HIG and hue > self.K_H_LOW and avgv < self.K_V_HIG and avgv > self.K_V_LOW:
            #     red_vote += 1
            if hue < self.Y_H_HIG and hue > self.Y_H_LOW:
                yellow_vote += 1
            if hue < self.G_H_HIG and hue > self.G_H_LOW:
                green_vote += 1
            if hue < self.B_H_HIG and hue > self.B_H_LOW:
                blue_vote += 1
            # if val < self.K_V_HIG and val > self.K_V_LOW:
            #     black_vote += .5
            # if val < self.B_V_HIG and val > self.B_V_LOW:
            #     blue_vote += .5
            # if val < self.R_V_HIG and val > self.R_V_LOW:
            #     red_vote += .5
            # if val < self.G_V_HIG and val > self.G_V_LOW:
            #     green_vote += .5
            # if val < self.Y_V_HIG and val > self.Y_V_LOW:
            #     yellow_vote += .5

        avgh, avgs, avgv = np.round(h_sum / tot), np.round(s_sum / tot), np.round(v_sum / tot)
        m = max(red_vote, black_vote, yellow_vote, green_vote, blue_vote)
        if np.round(s_sum / tot) == 0.0:
            m = -1

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
        if -1 == m:
            color = 'k'
        cv2.putText(draw, "{}: {}: {}: {}".format(color, avgh, avgs, avgv), (20, 20), 1, 2, (255, 0, 0))
        # img = np.repeat(image, 20, axis=0)
        # debug.add_image(img, "roi", topic="roi")
        # cv2.imshow("roi", img)
        # cv2.waitKey(0)
        debug.add_image(draw, "colors", topic="colors")
        return color

    def check_for_colors(self, image, colors, debug):
        color = self.get_color(image, colors, debug)
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
            if self.curr_count > 0 and (len(self.colors) == 0 or self.colors[len(self.colors) - 1] != self.prev_color):
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
