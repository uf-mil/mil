import cv2
import numpy as np
from collections import deque
import itertools
___author___ = "Tess Bianchi"

class MedianFlow(object):
    TRACKING_LENGTH = 3

    def __init__(self, elimination_amount=.6, winSize=(15, 15), maxLevel=2):
        self.prev_points = None
        self.prev_frame = None
        self.lk_params = dict(winSize=winSize, maxLevel=maxLevel, criteria=(
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 2, 0.03))
        self.tracking_points = deque()
        self.tracking_frames = deque()
        self.bboxs = deque()

        self.elimination_amount = elimination_amount
        self._is_effective = True

        self.bbox = None
        self._curr_scale = 1
        self._amount_mdn_flow_tracked = 0

    def init(self, frame, bbox, img=None, num_points=6):
        '''
        Arguments: num_points is the square root of the number of that will be initially given to your image
        '''
        if frame is None:
            raise TypeError("The frame is invalid")

        x, y, w, h = bbox

        if(w <= 0 or h <= 0):
            raise ValueError("Invalid bounding box")

        self.prev_frame = frame

        self.bbox = bbox

        px = np.linspace(x, x + w, num_points)
        py = np.linspace(y, y + h, num_points)
        px = px.astype(np.int32)
        py = py.astype(np.int32)
        p = np.array(list(itertools.product(px, py)), dtype=np.float32)
        p = p.reshape(p.shape[0], 2)

        self.prev_points = p

    def stop_use(self):
        self.tracking_points.clear()
        self.tracking_frames.clear()
        self.bboxs.clear()
        self._amount_mdn_flow_tracked = 0
        self._is_effective = True
        self.curr_scale = 1

    def is_effective(self):
        return self._is_effective

    def get_last_good_frame(self):
        best = self.bboxs.popleft()
        best_frame = self.tracking_frames.popleft()
        x, y, w, h = best
        _roi = best_frame[y:y + h, x:x + w]
        return _roi, best

    def _calculate_forward_backward_error(self, prev_frame, prev_points):
        # Get the last set of frame in the list, calulate optical flow from f(t) to f(t-TRACKING_LENGTH)
        # for the number of frame in tracking frame
        # Go backwards through the list because we want the most recent element first
        for i in range(self.TRACKING_LENGTH - 1, -1, -1):
            _frame = self.tracking_frames[i]
            _points, status, err = cv2.calcOpticalFlowPyrLK(prev_frame, _frame, prev_points, None, **self.lk_params)
            prev_frame = _frame
            prev_points = _points

        # We now have f(t-TRACKING_LENGTH), get the euclidean distance from each
        # of these points to the oldest points in tracking_points
        old_points = self.tracking_points.popleft()
        diff = np.subtract(old_points, prev_points)
        diff_norm = np.linalg.norm(diff, axis=1)

        return diff, diff_norm

    def _eliminate_points(self, points, frame):
        if frame is None:
            raise TypeError("The frame is invalid")

        # Make sure there is enough frames in our list
        if(len(self.tracking_points) < self.TRACKING_LENGTH):
            self.tracking_points.append(points)
            self.tracking_frames.append(frame)
            return points

        diff, diff_norm = self._calculate_forward_backward_error(frame, points)
        # Eliminate the points based on the fb error
        msk = None

        # If this is the first time median flow is tracking these points, eliminate the top 60% of the points with the highest
        # fb tracking error 60% is defined in self.elimination_amount)
        if(self._amount_mdn_flow_tracked == 0):
            lrgst_ind = np.argsort(diff_norm)[-int(len(diff_norm) * self.elimination_amount):]
            msk = np.ones(len(diff_norm), dtype=bool)
            msk[lrgst_ind] = False
        else:
            # Elimate points with a foward-backward error greater than .8
            high_val = diff_norm > .8
            diff = diff.T[0]
            msk = np.ones(len(diff_norm), dtype=bool)
            msk[high_val] = False

        # Eliminate these from the current points, also, from all previous points as well
        points = points[msk]
        for i, tp in enumerate(self.tracking_points):
            self.tracking_points[i] = tp[np.array(msk)]

        self.prev_points = self.tracking_points[self.TRACKING_LENGTH - 2]

        # Update tracking values
        self.tracking_points.append(points)
        self.tracking_frames.popleft()
        self.tracking_frames.append(frame)
        self._amount_mdn_flow_tracked += 1

        # Determine if this model is still effective
        # If the mean fb error is > 2 and the amount of frames tracked is > 20, then the model has failed.
        # Also if the number of points in the model is <d the model has failed
        if((cv2.mean(diff)[0] > 2 and self._amount_mdn_flow_tracked > 20) or len(points) < 3):
            self._is_effective = False

        return points

    def _update_bbox(self, curr_points):
        # Update bounding dimensions
        # Get the median dx and dy, this is how far the bounding box moves
        mydiff = np.subtract(curr_points, self.prev_points)
        a = mydiff.transpose()
        x_d = np.median(a[0])
        y_d = np.median(a[1])
        x, y, w, h = self.bbox

        x_n = int(round(x + x_d))
        y_n = int(round(y + y_d))

        # Scale is a little trickier, get every permutation of two points from previous points and current points
        prev_comb = list(itertools.permutations(self.prev_points, 2))
        curr_comb = list(itertools.permutations(curr_points, 2))
        ratios = []

        for i, val in enumerate(prev_comb):
            if(i >= len(curr_comb)):
                # This should never happen
                raise ValueError('The previous points and current points are not synchronized')
            prev_point_A = prev_comb[i][0]
            prev_point_B = prev_comb[i][1]
            curr_point_A = curr_comb[i][0]
            curr_point_B = curr_comb[i][1]

            # Get the distance between every corresponsing pair of points
            prev_dist = np.subtract(prev_point_A, prev_point_B)
            curr_dist = np.subtract(curr_point_A, curr_point_B)

            # Dont want to divide by zero
            if(np.linalg.norm(prev_dist) == 0):
                continue

            # Get the ration of the current distance, to the previous distance
            ratios.append(np.linalg.norm(curr_dist) / np.linalg.norm(prev_dist))

        # Choose the median of these distances
        scale = np.median(ratios)

        # Multiply the current scale with this scale
        self._curr_scale *= scale

        w_n = w
        h_n = h

        # If the scale is big or small enough (due to rounding errors)
        if(self._curr_scale > 1.08 or self._curr_scale < .92):
            w_n = int(round(w * self._curr_scale))
            h_n = int(round(h * self._curr_scale))
            self._curr_scale = 1

        if(x_n < 0 or y_n < 0 or w_n < 0 or h_n < 0):
            raise ValueError("The new bounding box has incorrect values")

        self.bbox = (x_n, y_n, w_n, h_n)

    def track(self, frame):
        if frame is None:
            raise TypeError("The frame is invalid")

        points, status, err = cv2.calcOpticalFlowPyrLK(self.prev_frame, frame, self.prev_points, None, **self.lk_params)
        points = self._eliminate_points(points, frame)
        try:
            self._update_bbox(points)
        except:
            return None

        self.bboxs.append(self.bbox)
        if(len(self.bboxs) > self.TRACKING_LENGTH):
            self.bboxs.popleft()

        self.prev_points = points
        self. prev_frame = frame
        return self.bbox