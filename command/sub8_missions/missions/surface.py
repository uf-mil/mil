from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Surfacing"
    yield sub_singleton.move.depth(0.6).go()
    print "Done surfacing"
