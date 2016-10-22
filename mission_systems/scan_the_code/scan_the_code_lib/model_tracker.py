import sys
import cv2
import numpy as np
import time
from model import Model


class ModelTracker:

    def __init__(self):
        self.model_count = 0
        self.models = []
        self.colors = []
        self.lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    def register_model(self, model, frame):
        # Go through all the models we already have
        # for i, m in enumerate(self.models):
        #     old_frame = m.prev_frame
        #     old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
        #     gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        #     prev_points = m.prev_points
        #     # calculate the point in the new image
        #     p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray, prev_points, None, **self.lk_params)

        #     # get the total distance between the points
        #     total_dists = 0
        #     for i, p in enumerate(p1):
        #         min_dist = 1000
        #         for j, _p in enumerate(model):
        #             dist = np.linalg.norm(np.subtract(p, _p))
        #             if(dist < min_dist):
        #                 min_dist = dist
        #         total_dists += min_dist

        #     # If the total distance is less than 50, you have a repeat model, don't add it
        #     if(total_dists < 50):
        #         return

        if(len(self.models) > 0):
            return

        model = Model(model, frame, self.model_count)
        self.models.append(model)
        self.model_count += 1

    def track_models(self, frame):
        draw = frame.copy()
        updated_models = []
        for i, m in enumerate(self.models):

            old_frame = m.prev_frame
            old_gray = cv2.cvtColor(old_frame, cv2.COLOR_BGR2GRAY)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            prev_points = m.prev_points

            p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray, gray, prev_points, None, **self.lk_params)
            for e in err:
                if(e == 1):
                    continue
                    print "err"
            if(not self.geometry_test(p1, gray.copy())):
                print "didn't pass geom"
                continue
            if(time.time() - m.time_started > 15):
                print "timed out"
                continue
            mission_status, colors = m.check_for_colors()
            if(mission_status):
                self.colors = colors
                return True

            m.prev_points = p1
            m.prev_frame = frame
            updated_models.append(m)

            for i, val in enumerate(m.prev_points):
                cv2.circle(draw, tuple(val[0]), 3, (255, 255, 255), -1)

        del self.models[:]
        self.models = updated_models

        if(len(self.models) > 0):
            cv2.imshow("all_models", draw)
            cv2.waitKey(33)

        return False

    def get_non_furthest_points(self, i, points):
        dist_points = []
        for ind, val in enumerate(points):
            if(ind != i):
                diff = np.subtract(points[i], val)
                dist_points.append([np.linalg.norm(diff), points[ind]])

        dist_points = sorted(dist_points, key=lambda x: x[0])

        return dist_points[0][1], dist_points[1][1]

    def geometry_test(self, fp, draw1):

        for i, p in enumerate(fp):
            draw = draw1.copy()
            p1, p2 = self.get_non_furthest_points(i, fp)
            cv2.circle(draw, tuple(p[0]), 3, (255, 255, 255), -1)
            cv2.circle(draw, tuple(p1[0]), 3, (255, 255, 255), -1)
            # cv2.circle(draw, tuple(p2[0]), 3, (255,255,255), -1)
            # cv2.imshow("geom", draw)
            # cv2.waitKey(0)

            diff1 = np.subtract(p1, p)[0]
            diff2 = np.subtract(p2, p)[0]

            diff1 /= np.linalg.norm(diff1)
            diff2 /= np.linalg.norm(diff2)

            val = abs(np.dot(diff1, diff2))
            if(val > .1):
                return False

        return True
