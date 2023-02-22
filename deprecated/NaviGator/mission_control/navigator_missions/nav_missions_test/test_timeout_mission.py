#!/usr/bin/env python
import axros
from mil_misc_tools.text_effects import fprint


@axros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    nh = navigator.nh
    attempts = kwargs["attempts"]
    fprint("{} running".format(__name__), msg_color="red")
    if attempts > 1:
        yield nh.sleep(1)
    else:
        yield nh.sleep(2)

    fprint("{} stopped running".format(__name__), msg_color="red")
