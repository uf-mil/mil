from txros import util
from sub8 import pose_editor

SIDE_LENGTH = 2  # meters
SPEED_LIMIT = 1  # m/s

@util.cancellableInlineCallbacks
def run(sub):
    print "Heil!"
    
    center = sub.move.forward(0)
    for i in range(4):
        forward = sub.move.forward(SIDE_LENGTH)
        right = forward.right(SIDE_LENGTH)

        yield forward.go(speed=SPEED_LIMIT)
        yield right.go(speed=SPEED_LIMIT)
        yield forward.go(speed=SPEED_LIMIT)
        yield center.go(speed=SPEED_LIMIT)
        yield sub.move.yaw_right_deg(90).go(speed=SPEED_LIMIT)

    print "Done!"
