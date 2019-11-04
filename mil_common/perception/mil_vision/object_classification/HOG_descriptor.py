import cv2


class HOGDescriptor(object):

    def __init__(self):
        self.hog = cv2.HOGDescriptor((8, 8), (8, 8), (4, 4), (8, 8), 9)

    def get_descriptor(self, img):
        return self.hog.compute(img)
