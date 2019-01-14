from txros import util


@util.cancellableInlineCallbacks
def run(sub_singleton):
    yield sub_singleton.move.zero_roll_and_pitch().go()
    print "Zero'd pitch and roll"
