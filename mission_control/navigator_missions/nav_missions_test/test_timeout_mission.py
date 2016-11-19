#!/usr/bin/env python
import txros
from navigator_tools import fprint
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    if np.random.rand() < .75:
        fprint("We gon timeout", msg_color='red')
        yield nh.sleep(10)
    else:
        fprint("We not gon timeout", msg_color='green')
        yield nh.sleep(2)
    fprint("{} stopped running".format(__name__), msg_color='red')
