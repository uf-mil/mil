from txros import util
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from hydrophones.msg import ProcessedPing

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.2
FREQUENCY = 30000
FREQUENCY_TOL = 500

PINGER_HEIGHT = 1.5  # how high to go above pinger

POSITION_TOL = 0.05


@util.cancellableInlineCallbacks
def run(sub):
    sub_position = yield sub.nh.subscribe('/hydrophones/processed',
                                          ProcessedPing)
    while True:
        p_message = yield sub_position.get_next_message()
        fprint(p_message)

        # Ignore freuqnecy
        if not abs(p_message.freq - FREQUENCY) < FREQUENCY_TOL:
            fprint("Ignored!", msg_color='red')
            continue

        p_position = mil_ros_tools.rosmsg_to_numpy(p_message.position)
        if abs(p_position[0]) < POSITION_TOL and abs(
                p_position[1] < POSITION_TOL):
            break
        vec = p_position / np.linalg.norm(p_position)
        transform = yield sub._tf_listener.get_transform(
            'base_link', 'hydrophones')
        vec = transform._q_mat.dot(vec)
        vec[2] = 0
        fprint(vec)
        yield sub.move.relative(vec).zero_roll_and_pitch().go(speed=SPEED)
        yield sub.nh.sleep(3)

    fprint('Arrived to hydrophones! Going down!')
    yield sub.move.to_height(PINGER_HEIGHT).zero_roll_and_pitch().go(speed=0.1)
