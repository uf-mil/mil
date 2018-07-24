from txros import util
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from hydrophones.msg import ProcessedPing
from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from twisted.internet import defer
import random

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

pub_cam_ray = None

SPEED = 0.2
FREQUENCY = 30000
FREQUENCY_TOL = 500

PINGER_HEIGHT = 1.5  # how high to go above pinger

POSITION_TOL = 0.05  # how close to pinger before quiting


@util.cancellableInlineCallbacks
def run(sub):

    fprint('Getting Guess Locations')

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
        pinger_guess = yield transform_to_baselink(sub, pinger_1_req,
                                                   pinger_2_req)
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
        if np.isnan(vec).any():
            fprint('Ignored! nan', msg_color='red')
            if use_prediction:
                pinger_guess = yield transform_to_baselink(
                    sub, pinger_1_req, pinger_2_req)
                where_to = random.choice(pinger_guess)
                where_to = where_to / np.linalg.norm(where_to)
                yield fancy_move(sub, where_to)
            continue
        transform = yield sub._tf_listener.get_transform(
            '/base_link', '/hydrophones')
        vec = transform._q_mat.dot(vec)

        fprint('Transformed vec: {}'.format(vec))

        # Check if we are on top of pinger
        if abs(vec[0]) < POSITION_TOL and abs(vec[1] < POSITION_TOL):
            break

        vec[2] = 0
        if use_prediction:
            pinger_guess = yield transform_to_baselink(sub, pinger_1_req,
                                                       pinger_2_req)
            fprint('Transformed guess: {}'.format(pinger_guess))
            # Check if the pinger aligns with guess
            check, vec = check_with_guess(vec, pinger_guess)

        yield fancy_move(sub, vec)

    fprint('Arrived to hydrophones! Going down!')
    yield sub.move.to_height(PINGER_HEIGHT).zero_roll_and_pitch().go(speed=0.1)


@util.cancellableInlineCallbacks
def fancy_move(sub, vec):
    yield sub.move.relative(vec).depth(0.5).zero_roll_and_pitch().go(
        speed=SPEED)
    yield sub.nh.sleep(3)


def check_with_guess(vec, pinger_guess):
    for guess in pinger_guess:
        guess[2] = 0
    dots = [vec.dot(guess / np.linalg.norm(guess)) for guess in pinger_guess]
    if dots[0] < -1 and dots[1] < -1:
        fprint(
            'Thought ping was behind. Dot: {}'.format(dots),
            msg_color='yellow')
        # Get the guess that is close to pnger vec
        go_to_guess = pinger_guess[np.argmin(map(abs, dots))]
        go_to_guess = go_to_guess / np.linalg.norm(go_to_guess)
        return (False, go_to_guess)
    return (True, vec)


@util.cancellableInlineCallbacks
def transform_to_baselink(sub, pinger_1_req, pinger_2_req):
    transform = yield sub._tf_listener.get_transform('/base_link', '/map')
    print(transform._p)
    pinger_guess = [
        transform._q_mat.dot(
            mil_ros_tools.rosmsg_to_numpy(x.location.pose.position) -
            transform._p) for x in (pinger_1_req, pinger_2_req)
    ]
    defer.returnValue(pinger_guess)
