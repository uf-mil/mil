#!/usr/bin/env python
import txros


@txros.util.cancellableInlineCallbacks
def main(navigator):
    pattern = navigator.move.circle_point([0,0,0], radius=5)
    searcher = navigator.search(vision_proxy='tester', search_pattern=pattern)

    yield searcher.start_search(spotings_req=1, speed=.9)