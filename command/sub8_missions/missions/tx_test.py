from txros import util


@util.cancellableInlineCallbacks
def run(sub):
    yield sub.nh.sleep(1)
    print "Goiung"
    yield sub.move.forward(-1).go()
