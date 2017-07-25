from txros import util
from mil_misc_tools import text_effects

FORWARD_METERS = 3.0


@util.cancellableInlineCallbacks
def run(sub):
    fprint = text_effects.FprintFactory(title='START_GATE (SIMPLE)').fprint
    fprint('Going forward')
    yield sub.move.zero_roll_and_pitch().forward(FORWARD_METERS).go()
    fprint('Done!')
