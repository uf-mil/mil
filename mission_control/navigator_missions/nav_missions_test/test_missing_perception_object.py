#!/usr/bin/env python
import txros
from navigator_tools import fprint


@txros.util.cancellableInlineCallbacks
def main(navigator):
    fprint("{} running".format(__name__), msg_color='red')
    resp = navigator.database_query("scan_the_code")
    
    yield resp
    if resp is None:
        fprint("Raising Perception Object", msg_color='red')
    else:
        fprint("{} stopped running".format(__name__), msg_color='red')
