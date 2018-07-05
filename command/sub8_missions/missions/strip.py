from txros import util
from mil_misc_tools import text_effects

SPEED_LIMIT = 5  # m/s

fprint = text_effects.FprintFactory(
    title="STRIPPER", msg_color="cyan").fprint


@util.cancellableInlineCallbacks
def pitch(sub):
    start = sub.move.forward(0).zero_roll_and_pitch()
    pitches = [start.pitch_down_deg(7), start] * 5
    for p in pitches:
        yield p.go(speed=0.1)


@util.cancellableInlineCallbacks
def run(sub):
    fprint('Starting...')
    start = sub.move.forward(0).zero_roll_and_pitch()
    fprint('Searching... pitching...')
    yield pitch(sub)
    fprint('Moving to gate')
    gate = start.forward(3)
    yield gate.go(speed=SPEED_LIMIT)
    fprint('Going past gate')
    yield sub.move.forward(3).go(speed=SPEED_LIMIT)
    fprint('Searching... pitching')
    yield pitch(sub)
    fprint('Going around pole')
    yield sub.move.forward(8).go(speed=SPEED_LIMIT)
    yield sub.move.left(1.7).go(speed=SPEED_LIMIT)
    yield sub.move.forward(3).go(speed=SPEED_LIMIT)
    yield sub.move.right(3.4).go(speed=SPEED_LIMIT)
    yield sub.move.backward(3).go(speed=SPEED_LIMIT)
    yield sub.move.left(1.7).go(speed=SPEED_LIMIT)

    fprint('Turning back to gate')
    yield sub.move.backward(1).go(speed=SPEED_LIMIT)
    fprint('Look at gate')
    yield sub.move.look_at(gate._pose.position).go(speed=SPEED_LIMIT)
    fprint('Going to gate')
    yield gate.yaw_left_deg(180).go(speed=SPEED_LIMIT)
    fprint('Go past through gate')
    yield start.yaw_left_deg(180).go(speed=SPEED_LIMIT)
