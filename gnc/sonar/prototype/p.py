#!/usr/bin/env python
from ctypes import *
lib = cdll.LoadLibrary('/home/tess/mil_ws/devel/lib/libsonar_tools.so')
import cv2

class Foo(object):
    def __init__(self):
        # s = "/home/tess/bvtsdk/data/swimmer.son"
        # s = "asdf"
        # d = c_char_p(s)
        print "a"
        # print d
        lib.newSonarFromFile.restype = c_char_p
        lib.newSonarFromFile.argtypes = [c_char_p]
        name = create_string_buffer("Tom")
        self.obj = lib.newSonarFromFile(name);

        # newSonarFromFile = lib.newSonarFromFile
        # newSonarFromFile.argtypes = [c_char_p]
        # self.obj = newSonarFromFile(b'hello')

    def getNextPing(self):
        return getNextPing(self.obj)

img = cv2.imread("/home/tess/zerba.jpg", 0)
lib.test(img)