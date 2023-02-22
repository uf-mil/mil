#!/usr/bin/env python
import axros
from mil_misc_tools.text_effects import fprint


@axros.util.cancellableInlineCallbacks
def main(navigator, **kwargs):
    fprint("STARTING EMPTY TEST", msg_color="green")
    yield navigator.nh.sleep(5)
    fprint("COMPLETED EMPTY TEST", msg_color="green")
