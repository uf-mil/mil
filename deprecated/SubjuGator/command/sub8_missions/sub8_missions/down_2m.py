from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    print "Dropping 2m"
    yield sub_singleton.move.down(2.0).go()
    print "Done!"
