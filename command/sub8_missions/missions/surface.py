from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    yield sub_singleton.move.depth(0.5).go()
