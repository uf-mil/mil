from txros import util
from mil_misc_tools import text_effects

SPEED_LIMIT = 0.15  # m/s

fprint = text_effects.FprintFactory(title="STRIPPER", msg_color="cyan").fprint


@util.cancellableInlineCallbacks
def pitch(sub):
    start = sub.move.forward(0).zero_roll_and_pitch()
    pitches = [start.pitch_down_deg(7), start] * 5
    for p in pitches:
        yield p.go(speed=0.1)


@util.cancellableInlineCallbacks
def run(sub):
    fprint('Starting...')
    yield sub.nh.sleep(5)
    yield sub.move.forward(0).zero_roll_and_pitch().go()
    yield sub.move.down(1).go(speed=0.1)
    yield sub.nh.sleep(3)
    start = sub.move.forward(0).zero_roll_and_pitch()
    # fprint('Searching... pitching...')
    # yield pitch(sub)
    fprint('Moving to gate')
    gate = start.forward(3)
    yield gate.go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(3)
    # fprint('Searching... pitching')
    # yield pitch(sub)
    fprint('Going right in front of pole')
    yield sub.move.forward(8.7).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(3)
    fprint('Going around pole')
    yield sub.move.left(1.3).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(3)
    yield sub.move.forward(2).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(3)
    yield sub.move.right(2.6).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(3)
    yield sub.move.backward(2).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(3)
    yield sub.move.left(1.3).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(5)

    fprint('Turning back to gate')
    yield sub.move.backward(1).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(5)
    fprint('Look at gate')
    yield sub.move.look_at(gate._pose.position).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(5)
    fprint('Going to gate')
    yield gate.yaw_left_deg(180).go(speed=SPEED_LIMIT)
    yield sub.nh.sleep(5)
    fprint('Go past through gate')
    yield start.yaw_left_deg(180).go(speed=SPEED_LIMIT)
