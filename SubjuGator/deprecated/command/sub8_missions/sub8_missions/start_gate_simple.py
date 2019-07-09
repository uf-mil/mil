from txros import util
from mil_misc_tools import text_effects

FORWARD_METERS = 3.0


@util.cancellableInlineCallbacks
def run(sub):
    fprint = text_effects.FprintFactory(title='START_GATE (SIMPLE)').fprint
    fprint('Remembering starting orientation')
    sub_start_position, sub_start_orientation = yield sub.tx_pose()
    fprint('Going down')
    down = sub.move.down(1)
    yield down.go(speed=0.1)
    fprint('Going forward')
    yield down.set_orientation(sub_start_orientation).zero_roll_and_pitch(
    ).forward(FORWARD_METERS).go(speed=0.2)
    fprint('Done!')
