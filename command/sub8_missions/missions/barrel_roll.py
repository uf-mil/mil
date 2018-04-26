from txros import util

SPEED_LIMIT = .3  # m/s


@util.cancellableInlineCallbacks
def run(sub):
    print "Square!"

    center = sub.move.forward(0).zero_roll_and_pitch()
    print "Centering"
    yield center.go(speed = SPEED_LIMIT)

    barrel = sub.move.roll_right_deg(180)
    print "Rolling! Upside down"
    yield barrel.go(speed = SPEED_LIMIT)
    print "Rolling! Going back"
    yield barrel.go(speed = SPEED_LIMIT)
    print "Done!"
