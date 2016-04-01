from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Dropping 5m"
    yield sub_singleton.move.down(5.0).go()
    print "Done!"
