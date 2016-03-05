from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    yield sub_singleton.move.right(5).go()
