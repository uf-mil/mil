from txros import util


@util.cancellableInlineCallbacks
def run(sub):
    yield sub.move.left(3).go()