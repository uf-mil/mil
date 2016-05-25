import cv2
import pickle
import numpy as np
import rospy
# from sub8_vision_tools.segmentation import bag_crawler
import bag_crawler
from matplotlib import pyplot as plt


class Picker(object):
    # Adaboost http://robertour.com/2012/01/24/adaboost-on-opencv-2-3/

    def __init__(self, image, brush_size=10, initial_mask=None):
        self.brush_size = brush_size
        self.done = False

        cv2.namedWindow("segment")
        self.image = image

        self.visualize = np.copy(image)
        if initial_mask is None:
            self.mask = np.zeros(image.shape[:2], dtype=np.uint8)
            self.mask[:, :] = int(cv2.GC_PR_BGD)
        else:
            self.mask = np.ones(image.shape[:2], dtype=np.uint8) * int(cv2.GC_PR_BGD)
            self.mask[initial_mask == int(cv2.GC_FGD)] = int(cv2.GC_FGD)
            self.mask[initial_mask == int(cv2.GC_BGD)] = int(cv2.GC_BGD)
            # self.mask = initial_mask
            self.visualize[self.mask == int(cv2.GC_FGD)] = (0, 200, 0)
            self.visualize[self.mask == int(cv2.GC_BGD)] = (0, 0, 200)

        self.mouse_state = [0, 0]
        cv2.setMouseCallback("segment", self.mouse_cb)
        cv2.imshow("segment", self.visualize)
        cv2.waitKey(1)

    def mouse_cb(self, event, x, y, flags, param):
        if self.done:
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.mouse_state[0] = 1
            cv2.circle(self.visualize, (x, y), self.brush_size, (0, 200, 0), -1)
            cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_FGD), -1)

        elif event == cv2.EVENT_LBUTTONUP:
            self.mouse_state[0] = 0

        elif event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_state[1] = 1
            cv2.circle(self.visualize, (x, y), self.brush_size, (0, 0, 200), -1)
            cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_BGD), -1)

        elif event == cv2.EVENT_RBUTTONUP:
            self.mouse_state[1] = 0

        elif event == cv2.EVENT_MBUTTONDOWN:
            print 'mbutton'

        elif event == cv2.EVENT_MOUSEMOVE:
            if self.mouse_state[0]:
                cv2.circle(self.visualize, (x, y), self.brush_size, (0, 200, 0), -1)
                cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_FGD), -1)

            elif self.mouse_state[1]:
                cv2.circle(self.visualize, (x, y), self.brush_size, (0, 0, 200), -1)
                cv2.circle(self.mask, (x, y), self.brush_size, int(cv2.GC_BGD), -1)

        cv2.imshow("segment", self.visualize)

    def wait_for_key(self, keys):
        print 'press one of:', keys
        while(not rospy.is_shutdown()):
            key = chr(cv2.waitKey(50) & 0xFF)
            if key in keys:
                return key

    def segment(self):
        key = self.wait_for_key(' qznw')
        if key in [' ', 'q']:
                return None, key
        self.done = True

        bgdModel = np.zeros((1, 65), np.float64)
        fgdModel = np.zeros((1, 65), np.float64)

        out_mask = np.copy(self.mask)

        print 'segmenting'
        cv2.grabCut(
            self.image,
            out_mask,
            None,
            bgdModel,
            fgdModel,
            5,
            cv2.GC_INIT_WITH_MASK
        )

        display_mask = np.ones(out_mask.shape).astype(np.float64)
        display_mask[(out_mask == cv2.GC_PR_BGD) | (out_mask == cv2.GC_BGD)] = 0.2

        display = display_mask[:, :, np.newaxis] * self.image.astype(np.float64)
        cv2.imshow('segment', display / np.max(display))

        # key = chr(cv2.waitKey(1000) & 0xFF)
        key = self.wait_for_key('qnzw ')
        if key in [' ', 'q']:
                return None, key

        display_int = (255 * display) / np.max(display).astype(np.uint8)
        return out_mask, key

    def save(self, mask):
        data = {"image": self.image, 'classes': mask}
        pickle.dump(data, open("save.p", "wb"))


if __name__ == '__main__':
    # img = cv2.imread('/home/jacob/Downloads/buoy.jpg')[::10, ::10, :]
    bag = '/home/jacob/catkin_ws/src/Sub8/gnc/sub8_perception/data/bag_test.bag'
    bc = bag_crawler.BagCrawler(bag)

    data = []

    last_mask = None
    for image in bc.crawl(topic=bc.image_topics[0]):
        p = Picker(image, brush_size=3, initial_mask=last_mask)
        last_mask, key = p.segment()
        if key == 'q':
            break
        elif key == ' ':
            print 'ignoring image'
            continue

        print 'recording'
        data.append((image, last_mask))

    pickle.dump(data, open('segments.p', 'wb'))

    # p.save(mask)
