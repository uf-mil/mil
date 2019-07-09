import txros
from twisted.internet import defer
from txros import util
from mil_misc_tools.text_effects import fprint, CvDebug
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
from collections import Counter
import numpy.ma as ma

# Perception
# Params : direction


class FindTheBreakPerception(object):

    def __init__(self, nh):
        """Initialize FindTheBreakPerception class."""
        # PARAMS
        self.diff_thresh = 50
        self.target_wh_ratio = .2
        self.max_from_target_ratio = .3

        self.nh = nh
        self.hpipe_position = None
        self.curr_image = None
        self.bridge = CvBridge()
        self.old_hpipe_pos = []
        self.old_vpipe_pos = []
        self.hpipe_found = False
        self.count = 0
        self.debug = CvDebug(nh=nh)

    @util.cancellableInlineCallbacks
    def init_(self):
        """Initialize the txros aspect of FindTheBreakPerception."""
        self._image_sub = yield self.nh.subscribe("/camera/down/image_rect_color", Image, lambda x: x)

    @property
    @util.cancellableInlineCallbacks
    def _curr_image(self):
        img_msg = yield txros.util.wrap_timeout(self._image_sub.get_next_message(), 3)
        defer.returnValue(self.bridge.imgmsg_to_cv2(img_msg, 'bgr8'))

    def _get_all_pipes(self, frame):
        frame
        h, w, r = frame.shape
        if h == 0:
            return

        # resize image
        frame = cv2.resize(frame, 0, fx=.2, fy=.2)
        gaussian = cv2.GaussianBlur(frame, (9, 9), 10.0)
        frame = cv2.addWeighted(frame, 1.5, gaussian, -0.5, 0, frame)

        nh, nw, r = frame.shape

        # cluster
        Z = frame.reshape((-1, 3))
        Z = np.float32(Z)
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        K = 5
        ret, label, center = cv2.kmeans(Z, K, criteria, 10, 0)
        center = np.uint8(center)
        image_as_centers = center[label.flatten()]
        image_as_centers = image_as_centers.reshape((frame.shape))
        labels = label.reshape((frame.shape[:2]))

        # Get the bounding boxes of the clusters
        rects = []

        for i in range(0, K):
            ind = np.where(labels == i)
            points = ind
            points = [[points[1], points[0]]]
            rect = cv2.minAreaRect(np.array(points).T)
            rects.append(rect)

        draw = image_as_centers.copy()
        draw1 = draw.copy()
        vrects = []
        hrects = []
        for r in rects:
            # Get the width and height of the box
            box = cv2.boxPoints(r)
            box = np.int0(box)
            cv2.drawContours(draw, [box], 0, (0, 0, 255), 2)
            p0 = box[0]
            p1 = box[1]
            vec1 = np.array(box[2] - p0)
            vec1 = vec1 / np.linalg.norm(vec1)
            vec2 = np.array(p1 - p0)
            vec2 = vec2 / np.linalg.norm(vec2)
            vec3 = np.array(box[3] - p0)
            vec3 = vec3 / np.linalg.norm(vec3)
            ang1 = np.arccos((vec1).dot(vec2))
            ang2 = np.arccos((vec3).dot(vec2))
            dif1 = 1.5708 - ang1
            dif2 = 1.5708 - ang2
            if dif1 < dif2:
                p2 = box[2]
            else:
                p2 = box[3]
            l, lp = np.linalg.norm(abs(p1 - p0)), p1
            w, wp = np.linalg.norm(abs(p2 - p0)), p2
            if l < w:
                temp = w
                templ = wp
                w = l
                wp = lp
                l = temp
                wp = templ

            # get the ratio
            rat = w / l
            diff = abs(rat - self.target_wh_ratio)
            if diff > self.max_from_target_ratio:
                continue

            horiz = False

            direc = (wp - p0) / np.linalg.norm(wp - p0)
            dot = direc.dot(np.array([1, 0]))
            vcost = abs(1 - dot)
            hcost = dot
            if hcost < .1:
                horiz = True
            elif vcost < .1:
                horiz = False
            else:
                continue

            # get percentage of pixels in the box
            box = cv2.boxPoints(r)
            box = np.int0(box)
            mask = np.zeros((nh, nw))
            cv2.drawContours(mask, [box], 0, 255, -1)
            myl = ma.array(labels, mask=mask)
            myl = myl[myl.mask].data
            c = Counter(myl).items()
            val = max(c, key=lambda x: x[1])
            val = val[1]
            frac = val / myl.size
            if frac < .8:
                continue

            cv2.drawContours(draw1, [box], 0, (0, 0, 255), 2)
            if horiz:
                hrects.append(r)
            else:
                vrects.append(r)

        self.debug.add_image(draw, "all_rects", topic="all_rects")
        self.debug.add_image(draw1, "real_rects", topic="real_rects")
        return hrects, vrects

    def _update_pipes(self, pipes, old_pipes):
        new_pipes = []
        if len(old_pipes) == 0:
            return pipes
        for i, h in enumerate(pipes):
            dists = map(lambda x: np.linalg.norm(h - x), old_pipes)
            pipe_to_update = min(old_pipes, key=lambda x: np.linalg.norm(h - x))
            idx = old_pipes.index(pipe_to_update)
            if dists[idx] < self.diff_thresh:
                old_pipes[idx] = h
            else:
                new_pipes.append(h)
        return new_pipes

    @util.cancellableInlineCallbacks
    def count_pipes(self):
        """Count the number of pipes in between the breaks."""
        second_hpipe_found = False
        while not second_hpipe_found:
            try:
                frame = yield self.curr_image
            except util.TimeoutError:
                fprint("Image isn't being received, fail mission", msg_color="red")
                raise Exception("Image isn't being received")

            # get all the pipes in the current frame
            hpipes, vpipes = self._get_all_pipes(frame)
            # Look for NEW pipes
            new_hpipes, new_vpipes = self._update_pipes(
                hpipes, self.old_hpipe_pos), self._update_pipes(vpipes, self.old_vpipe_pos)

            # the second hpipe is found
            if len(new_hpipes) > 0 and self.hpipe_found:
                defer.returnValue(self.count)

            # the first hpipe is found
            elif len(new_hpipes) > 0 and not self.hpipe_found:
                self.hpipe_found = True
                for pipe in vpipes:
                    if pipe[1] > new_hpipes[0][1]:
                        self.count += 1

            # if its above first h, add to count
            elif self.hpipe_found:
                for pipe in new_vpipes:
                    if pipe[1] > new_hpipes[0][1]:
                        self.count += 1

            self.old_hpipe_pos.extend(new_hpipes)
            self.old_vspipe_pos.extend(new_vpipes)
