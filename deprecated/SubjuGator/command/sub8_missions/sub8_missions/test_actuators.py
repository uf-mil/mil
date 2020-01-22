from txros import util
from mil_misc_tools import FprintFactory

failed = False


@util.cancellableInlineCallbacks
def run(sub):
    fprint = FprintFactory(title='ACUTATOR TEST').fprint

    @util.cancellableInlineCallbacks
    def test():
        yield sub.actuators.gripper_open()
        yield sub.actuators.gripper_close()
        yield sub.actuators.shoot_torpedo1()
        yield sub.actuators.shoot_torpedo2()
        yield sub.actuators.drop_marker()
        yield sub.actuators.set('gripper', False)
        yield sub.actuators.open('torpedo1')
        yield sub.actuators.set_raw('torpedo1', True)

    def err(err):
        fprint('Error with actuators: {}'.format(err), msg_color='red')
        global failed
        failed = True

    test_runner = test()
    test_runner.addErrback(err)
    yield test_runner
    global failed
    if not failed:
        fprint('Success with all actuators!', msg_color='blue')
    else:
        fprint('Failed 1 or more actuator calls', msg_color='red')
