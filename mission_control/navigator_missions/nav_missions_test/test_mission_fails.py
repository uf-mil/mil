#!/usr/bin/env python
import txros
from mil_misc_tools.text_effects import fprint


@txros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    nh = navigator.nh
    fprint("{} running".format(__name__), msg_color='red')
    yield nh.sleep(2)
    fprint("{} stopped running, raising exception".format(__name__), msg_color='red')
    raise Exception()


def safe_exit(navigator, err):
    fprint("SAFE EXIT", msg_color="blue")
