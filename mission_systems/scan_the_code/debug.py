import cv2
import numpy as np


class Debug(object):

    def __init__(self, w=1000, h=800, total=8, win_name="debug", wait=True):
        self.width = w
        self.height = h
        self.img = np.zeros((h, w, 3), np.uint8)
        self.total = total
        self.hor_num = total / 2
        self.vert_num = 2
        self.max_width = w / self.hor_num
        self.max_height = h / self.vert_num
        self.wait = wait

        self.curr_w = 0
        self.curr_h = 0
        self.num_imgs = 0
        self.win_name = win_name
        self.name_to_starting = {}

    def add_image(self, img, name, wait=33):
        if self.wait:
            wait = 0
        if len(img.shape) == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        h, w, r = img.shape
        if w > h and w > self.max_width:
            img = cv2.resize(img, (self.max_width, h * self.max_width / w))

        if h > w and h > self.max_height:
            img = cv2.reshape(img, (w * self.max_height / h, self.max_height))
        h, w, r = img.shape
        if name not in self.name_to_starting:
            if self.num_imgs == self.total:
                print "Too many images"
                return
            self.name_to_starting[name] = (self.curr_w, self.curr_h)
            self.num_imgs += 1

            self.curr_w += w
            if self.curr_w > self.width:
                self.curr_w = 0
                self.curr_h = self.max_height
        my_w, my_h = self.name_to_starting[name]
        self.img[my_h: my_h + h, my_w: my_w + w] = img
        cv2.imshow(self.win_name, self.img)
        cv2.waitKey(wait)

    def show(self, name=None, wait=33):
        if self.wait:
            wait = 0
        if name is None:
            name = self.win_name
        cv2.imshow(name, self.img)
        cv2.waitKey(wait)



 # average the height of these lines, and the dir vec
        avg_height = 0
        average_dirvec = (0, 0)
        for line in lines:
            min_y, y, x = line
            average_dirvec = [sum(a) for a in zip(make_vec((x, y)), average_dirvec)]
            avg_height += min_y
        avg_height /= len(line)
        print average_dirvec
        average_dirvec = map(lambda x: x / len(lines), average_dirvec)
        print average_dirvec