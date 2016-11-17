#!/usr/bin/env python
import txros
import numpy as np


@txros.util.cancellableInlineCallbacks
def main(navigator):
    res = yield navigator.move.circle_point([5, 0]).go()
