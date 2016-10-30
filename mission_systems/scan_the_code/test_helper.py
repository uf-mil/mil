from bag_crawler import BagCrawler
import argparse
import cv2
import sys


class TestHelper(object):

    def __init__(self):
        self.folder = 'testing'

    def get_roi(self, name):
        file = open(self.folder + '/' + name, 'r')
        for line in file:
            line = line.replace('\n', '')
            if len(line) == 0:
                continue
            x, y, w, h = line.split(" ")
            yield x, y, w, h

    def create(self, bag, output, topic='/stereo/left/image_raw'):
        bc = BagCrawler(bag)
        self.crawl_bu = bc.crawl(topic=topic)
        image = self.crawl_bu.next()
        self.crawl = bc.crawl(topic=topic)
        self.x, self.y = None, None
        self.w, self.h = 0, 0
        w, h, r = image.shape
        self.button_pressed = False
        self.output = open(self.folder + "/" + output, 'w+')
        self.window_name = 'segment'
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_roi)
        cv2.createTrackbar("width", self.window_name, self.w, w, lambda x: setattr(self, 'w', x))
        cv2.createTrackbar("height", self.window_name, self.h, h, lambda x: setattr(self, 'h', x))

    def go(self):
        while self.x is None:
            cv2.waitKey(33)
        first = True
        image = None
        while True:
            k = cv2.waitKey(33)
            if k == 1048608 or first:
                try:
                    image = self.crawl.next()
                except StopIteration:
                    break
                clone = image.copy()
                cv2.rectangle(clone, (self.x - self.w / 2, self.y - self.h / 2),
                              (self.x + self.w / 2, self.y + self.h / 2), (0, 255, 0))
                cv2.imshow(self.window_name, clone)
                cv2.waitKey(33)
                self.output.write('{} {} {} {}\n'.format(self.x, self.y, self.w, self.h))
                first = False
            elif k == 1048689:
                self.output.close()
                break
            else:
                clone = image.copy()
                cv2.rectangle(clone, (self.x - self.w / 2, self.y - self.h / 2),
                              (self.x + self.w / 2, self.y + self.h / 2), (0, 255, 0))
                cv2.imshow(self.window_name, clone)
                cv2.waitKey(33)

    def mouse_roi(self, event, x, y, flags, params):
        self.x = x
        self.y = y


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('bag', type=str, help='The bag you would like to use')
    parser.add_argument('name', type=str, help='The name of the output file')
    args = parser.parse_args(sys.argv[1:])

    testhelper = TestHelper()
    testhelper.create(args.bag, args.name)
    testhelper.go()
