from txros import util
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from hydrophones.msg import ProcessedPing
from sub8_msgs.srv import GuessRequest, GuessRequestRequest

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.2
FREQUENCY = 30000
FREQUENCY_TOL = 500

PINGER_HEIGHT = 1.5  # how high to go above pinger

POSITION_TOL = 0.05  # how close to pinger before quiting


@util.cancellableInlineCallbacks
def run(sub):

    pinger_txros = yield sub.nh.get_service_client('/guess_location',
                                                   GuessRequest)
    pinger_1_req = yield pinger_txros(GuessRequestRequest(item='pinger1'))
    pinger_2_req = yield pinger_txros(GuessRequestRequest(item='pinger2'))

    use_prediction = True
    if pinger_1_req.found is False or pinger_2_req.found is False:
        use_prediction = False
        fprint('Forgot to add pinger to guess server?', msg_color='yellow')
    else:
        fprint('Found two pinger guesses', msg_color='green')
        transform = yield sub._tf_listener.get_transform(
            'base_link', '/map')
        pinger_guess = [
            transform._q_mat.dot(
                mil_ros_tools.rosmsg_to_numpy(x.location.pose.position))
            for x in (pinger_1_req, pinger_2_req)
        ]
        fprint(pinger_guess)

    hydro_processed = yield sub.nh.subscribe('/hydrophones/processed',
                                             ProcessedPing)

    while True:
        p_message = yield hydro_processed.get_next_message()

        # Ignore freuqnecy
        if not abs(p_message.freq - FREQUENCY) < FREQUENCY_TOL:
            fprint("Ignored!", msg_color='red')
            continue

        # Transform frames
        p_position = mil_ros_tools.rosmsg_to_numpy(p_message.position)
        vec = p_position / np.linalg.norm(p_position)
        transform = yield sub._tf_listener.get_transform(
            'base_link', 'hydrophones')
        vec = transform._q_mat.dot(vec)

        fprint(vec)

        # Check if we are on top of pinger
        if abs(vec[0]) < POSITION_TOL and abs(vec[1] < POSITION_TOL):
            break

        vec[2] = 0
        # Check if the pinger aligns with guess
        if use_prediction and check_with_guess(vec, pinger_guess) is False:
            continue
        yield sub.move.relative(vec).zero_roll_and_pitch().go(speed=SPEED)
        yield sub.nh.sleep(3)

    fprint('Arrived to hydrophones! Going down!')
    yield sub.move.to_height(PINGER_HEIGHT).zero_roll_and_pitch().go(speed=0.1)


def check_with_guess(vec, pinger_guess):
    for guess in pinger_guess:
        guess[2] = 0
        dot = vec.dot(guess)
        if dot < -1:
            fprint('Thought ping was behind. Dot: {}'.format(dot))
            return False
    return True
