from roi_generator import ROI_Collection
from navigator_tools import Debug, BagCrawler
import numpy as np
from HOG_descriptor import HOGDescriptor
from SVM_classifier import SVMClassifier
import pickle
import cv2
from cv_bridge import CvBridge
___author___ = "Tess Bianchi"


class Config(object):

    def __init__(self):
        self.classes = ["totem", "scan_the_code", "nothing", "shooter"]
        self.classifier = SVMClassifier()
        self.descriptor = HOGDescriptor()
        self.bridge = CvBridge()
        self.MAX_SIZE = 74
        self.IMAGE_SIZE = 100

    def to_class(self, val):
        return self.classes[val]

    def to_val(self, clss):
        for i, c in enumerate(self.classes):
            if c in clss:
                return i

    def get_imgs(self, val):
        roi = pickle.load(open(val, 'rb'))
        imgs = []
        rois_vals = []
        rois = []
        print roi

        for b in roi.bag_to_rois.keys():
            frames = roi.bag_to_rois[b]
            bc = BagCrawler(b)
            topic = bc.image_topics[0]
            bc_crawl = bc.crawl(topic)
            print b
            for frame in frames:
                img = bc_crawl.next()
                img = self.bridge.imgmsg_to_cv2(img, 'bgr8')
                imgs.append(img)
                a = []
                for clss in frame:
                    r = frame[clss]
                    myroi = img[r[1]:r[1] + r[3], r[0]:r[0] + r[2]]
                    myroi = self._resize_image(myroi)
                    clss = self.to_val(clss)
                    rois.append((myroi, clss))
                    a.append((r, myroi, clss))
                rois_vals.append(a)
        return imgs, rois_vals, rois

    def _resize_image(self, img):
        h, w, r = img.shape
        if h > w:
            nh = self.MAX_SIZE
            nw = nh * w / h
        else:
            nw = self.MAX_SIZE
            nh = nw * h / w
        img = cv2.resize(img, (nw, nh))
        # return img
        rep = np.ones(nw, dtype=np.int64)
        reph = np.ones(nh, dtype=np.int64)
        emtpy_slots = self.IMAGE_SIZE - nw
        empty_slots_h = self.IMAGE_SIZE - nh
        half_empty_slots = emtpy_slots / 2 + 1
        half_empty_slots_h = empty_slots_h / 2 + 1
        reph[0] = half_empty_slots_h
        reph[-1] = half_empty_slots_h
        rep[0] = half_empty_slots
        rep[-1] = half_empty_slots
        if emtpy_slots % 2 == 1:
            rep[-1] += 1

        if empty_slots_h % 2 == 1:
            reph[-1] += 1

        img = np.repeat(img, reph, axis=0)
        return np.repeat(img, rep, axis=1)


class Training(object):

    def __init__(self, roi_file, output):
        self.config = Config()
        self.output = output
        self.roi_file = roi_file

    def train(self):
        descs = []
        classify = []
        imgs, roi_val, rois = self.config.get_imgs(self.roi_file)
        for r in rois:
            roi, clss = r
            desc = self.config.descriptor.get_descriptor(roi)
            desc = desc.flatten()
            descs.append(desc)
            classify.append(clss)
        descs = np.array(descs)
        classify = np.array(classify)
        counts = dict((x, list(classify).count(x)) for x in set(classify))
        counts = dict((self.config.to_class(k), v) for k, v in counts.items())
        print counts

        self.config.classifier.train(descs, classify)
        self.config.classifier.pickle("train.p")

# class Classifier(object):


class ClassiferTest(object):

    def __init__(self, roi_file, class_file):
        self.config = Config()
        self.roi_file = roi_file
        self.classifier = pickle.load(open(class_file, 'rb'))
        self.debug = Debug()

    def classify(self):
        print self.roi_file
        imgs, roi_val, rois = self.config.get_imgs(self.roi_file)
        print "dfd"
        for i, frames in enumerate(roi_val):
            img = imgs[i]
            draw = img.copy()
            for roi in frames:
                myroi, roi_img, tru_clss = roi
                desc = self.config.descriptor.get_descriptor(roi_img)
                desc = desc.flatten()
                clss, prob = self.classifier.classify(desc)
                clss = self.config.to_class(clss)
                cv2.rectangle(draw, (myroi[0], myroi[1]), (myroi[0] + myroi[2], myroi[1] + myroi[3]), (0, 0, 255))
                cv2.putText(draw, clss + ": " + str(prob), (myroi[0], myroi[1]), 1, 1.0, (0, 255, 0))
            # self.debug.add_image(draw, topic="roi")
            cv2.imshow("roi", draw)
            cv2.waitKey(33)

if __name__ == "__main__":
    # t = Training("train_roi.p", "train.p")
    # t.train()
    # print "done"
    c = ClassiferTest("val_roi.p", "train.p")
    c.classify()
