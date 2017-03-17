from __future__ import division
from roi_generator_slow import ROI_Collection_Slow
import pickle
import numpy as np
import cv2
import numpy.linalg as npl
import numpy.ma as ma
from navigator_tools import BagCrawler
from cv_bridge import CvBridge
import scipy.ndimage.measurements as mes
from SVM_classifier import SVMClassifier


def _get_lw(box):
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
        lp = templ
    direc = (wp - p0) / np.linalg.norm(wp - p0)
    dot = direc.dot(np.array([0, 1]))
    vcost = abs(dot)
    return l, w, vcost


def get_rectangle(roi):
    """
    Get the rectangle that has changing colors in the roi.

    Returns boolean success value and the four rectangle points in the image
    """
    gaussian = cv2.GaussianBlur(roi, (9, 9), 10.0)
    roi = cv2.addWeighted(roi, 1.5, gaussian, -0.5, 0, roi)

    nh, nw, r = roi.shape

    # cluster
    Z = roi.reshape((-1, 3))
    Z = np.float32(Z)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
    K = 7
    ret, label, centers = cv2.kmeans(Z, K, criteria, 10, 0)
    centers = np.uint8(centers)
    image_as_centers = centers[label.flatten()]
    image_as_centers = image_as_centers.reshape((roi.shape))
    labels = label.reshape((roi.shape[:2]))

    possible_clusters = list(np.arange(K))
    whiteness = map(lambda x: npl.norm(x - np.array([255, 255, 255])), centers)
    whitest = np.argmin(whiteness)
    possible_clusters.remove(whitest)
    energys = []
    correct_masks = []
    for num, p in enumerate(possible_clusters):
        mask_clusters = ma.masked_equal(labels, p)

        draw_mask = mask_clusters.mask.astype(np.uint8)
        draw_mask *= 255

        labeled_array, num_features = mes.label(draw_mask)
        count = np.bincount(labeled_array.flatten())
        count = count[1:]
        val = np.argmax(count)
        mask_obj = ma.masked_equal(labeled_array, val + 1)

        draw_mask = mask_obj.mask.astype(np.uint8)
        draw_mask *= 255

        # cv2.imshow(str(num), draw_mask)
        # cv2.waitKey(0)

        top = np.count_nonzero(draw_mask)
        valz = np.fliplr(np.transpose(draw_mask.nonzero()))
        rect = cv2.minAreaRect(valz)
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        rect_mask = np.zeros((nh, nw))
        cv2.drawContours(rect_mask, [box], 0, 255, -1)
        bottom = np.count_nonzero(rect_mask)

        l, w, vcost = _get_lw(box)
        if w < .001:
            print 'WIDTH TOO SMALL'
            continue

        valz = np.fliplr(np.transpose(draw_mask.nonzero()))
        area = cv2.contourArea(box)
        area /= (nh * nw)

        if vcost > .5:
            print "VCOST TOO HIGH"
            continue
        if area < .03:
            print area
            print "TOOOO SMALL"
            continue
        if top / bottom < .7:
            print "TOO SPARSE", top / bottom
            continue

        energy = area + 1.5 * top / bottom - abs(2.5 - l / w) - .2 * vcost
        if energy < 0:
            "LOW ENERGY!"
            continue
        print num, "area: ", area, "filled:", top, "total:", bottom, 'rat', top / bottom, "l/w", abs(2.5 - l / w), "vcost",
        vcost, "energy", energy
        energys.append(energy)
        correct_masks.append(mask_obj)

    if len(energys) == 0:
        print "EVERY ENERGY WRONG"
        return False, None

    correct_masks = [x for y, x in sorted(zip(energys, correct_masks), reverse=True)]
    energys = sorted(energys, reverse=True)

    if len(energys) > 1 and abs(energys[0] - energys[1]) < .2:
        print "TOO CLOSE TO CALLS"
        return False, None

    correct_mask = correct_masks[0]
    colors = roi[correct_mask.mask]
    draw_mask = correct_mask.mask.astype(np.uint8)
    draw_mask *= 255

    return True, colors


class Config():

    def __init__(self):
        self.mymap = {'r': 1, 'b': 2, 'y': 3, 'k': 4}
        self.inv_map = {v: k for k, v in self.mymap.iteritems()}

    def get_class(self, val):
        return self.inv_map[val]

    def get_val(self, clss):
        return self.mymap[clss]


class Training(object):

    def __init__(self):
        self.config = Config()
        self.svm = SVMClassifier()
        self.data = []
        self.colors = []

    def train(self, img, colors, color):
        mean = np.mean(np.mean(img, axis=0), axis=0)
        m = np.repeat([mean], len(colors), axis=0)
        t = np.hstack([colors, m])
        color = self.config.get_val(color)
        c = np.repeat(color, len(colors))

        self.data.extend(t)
        self.colors.extend(c)

    def pickle(self, file):
        self.svm.train(self.data, self.colors)
        self.svm.pickle(file)


if __name__ == "__main__":
    t = Training()
    bridge = CvBridge()
    pick = pickle.load(open("stc_train1.p", 'rb'))
    for bag in pick.bag_to_rois:
        colors = pick.bag_to_rois[bag]
        b = BagCrawler(bag)
        topic = b.image_topics[0]
        crawl = b.crawl(topic=topic)
        for color in colors:
            if len(color) is 0:
                continue
            color, roi = color.iteritems().next()
            img = crawl.next()
            img = bridge.imgmsg_to_cv2(img, 'bgr8')
            image_clone = img.copy()
            print roi, color
            xmin, ymin, w, h = roi[0], roi[1], roi[2], roi[3]
            cv2.rectangle(image_clone, (xmin, ymin), (xmin + w, ymin + h), (0, 255, 0), 2)
            roi = img[ymin:ymin + h, xmin:xmin + w]
            succ, color_vec = get_rectangle(roi)
            if succ:
                t.train(img, color_vec, color)
    t.pickle("/home/tess/mil_ws/src/NaviGator/mission_systems/navigator_scan_the_code/navigator_scan_the_code/scan_the_code_lib/svm_train.p")
