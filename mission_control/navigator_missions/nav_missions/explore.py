#!/usr/bin/env python
import txros
import tf
import numpy as np
import navigator_tools
from twisted.internet import defer

@txros.util.cancellableInlineCallbacks
def main(navigator):
    navigator.nh.get_service_cleint("/explorer/get_next_point",)