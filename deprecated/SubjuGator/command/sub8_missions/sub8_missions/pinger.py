import random

import mil_ros_tools
import numpy as np
import visualization_msgs.msg as visualization_msgs
from geometry_msgs.msg import Point, Vector3
from mil_misc_tools import text_effects
from sub8_hydrophones.msg import ProcessedPing
from subjugator_msgs.srv import GuessRequest, GuessRequestRequest
from twisted.internet import defer
from axros import util
from visualization_msgs.msg import Marker, MarkerArray

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 0.1
FREQUENCY = 30000
FREQUENCY_TOL = 3000

PINGER_HEIGHT = 1.5  # how high to go above pinger after found
MOVE_AT_DEPTH = 1.5  # how low to swim and move

POSITION_TOL = 0.05  # how close to pinger before quiting


@util.cancellableInlineCallbacks
def run(sub):
    global markers
    markers = MarkerArray()
    pub_markers = yield sub.nh.advertise("/pinger/rays", MarkerArray)

    fprint("Getting Guess Locations")

    pinger_axros = yield sub.nh.get_service_client("/guess_location", GuessRequest)
    pinger_1_req = yield pinger_axros(GuessRequestRequest(item="pinger1"))
    pinger_2_req = yield pinger_axros(GuessRequestRequest(item="pinger2"))

    use_prediction = True
    if pinger_1_req.found is False or pinger_2_req.found is False:
        use_prediction = False
        fprint("Forgot to add pinger to guess server?", msg_color="yellow")
    else:
        fprint("Found two pinger guesses", msg_color="green")
        pinger_guess = yield transform_to_baselink(sub, pinger_1_req, pinger_2_req)
        fprint(pinger_guess)

    hydro_processed = yield sub.nh.subscribe("/hydrophones/processed", ProcessedPing)

    while True:
        fprint("=" * 50)
        pub_markers.publish(markers)
        p_message = yield hydro_processed.get_next_message()

        # Ignore freuqnecy
        if not abs(p_message.freq - FREQUENCY) < FREQUENCY_TOL:
            fprint(
                "Ignored! Recieved Frequency {}".format(p_message.freq), msg_color="red"
            )
            if use_prediction:
                yield go_to_random_guess(sub, pinger_1_req, pinger_2_req)
            continue

        # Ignore magnitude from processed ping
        p_position = mil_ros_tools.rosmsg_to_numpy(p_message.position)
        vec = p_position / np.linalg.norm(p_position)
        if np.isnan(vec).any():
            fprint("Ignored! nan", msg_color="red")
            if use_prediction:
                yield go_to_random_guess(sub, pinger_1_req, pinger_2_req)
            continue

        # Tranform hydrophones vector to relative coordinate
        transform = yield sub._tf_listener.get_transform("/base_link", "/hydrophones")
        vec = transform._q_mat.dot(vec)

        fprint("Transformed vec: {}".format(vec))
        marker = Marker(
            ns="pinger",
            action=visualization_msgs.Marker.ADD,
            type=Marker.ARROW,
            scale=Vector3(0.2, 0.5, 0),
            points=np.array([Point(0, 0, 0), Point(vec[0], vec[1], vec[2])]),
        )
        marker.id = 3
        marker.header.frame_id = "/base_link"
        marker.color.r = 1
        marker.color.g = 0
        marker.color.a = 1
        markers.markers.append(marker)

        # Check if we are on top of pinger
        if abs(vec[0]) < POSITION_TOL and abs(vec[1] < POSITION_TOL):
            sub_position, _ = yield sub.tx_pose()
            dists = [
                np.linalg.norm(
                    sub_position
                    - mil_ros_tools.rosmsg_to_numpy(x.location.pose.position)
                )
                for x in (pinger_1_req, pinger_2_req)
            ]
            pinger_id = np.argmin(dists)
            yield sub.nh.set_param("pinger_where", int(pinger_id))

            break

        vec[2] = 0
        if use_prediction:
            pinger_guess = yield transform_to_baselink(sub, pinger_1_req, pinger_2_req)
            fprint("Transformed guess: {}".format(pinger_guess))
            # Check if the pinger aligns with guess
            check, vec = check_with_guess(vec, pinger_guess)

        fprint("move to {}".format(vec))
        yield fancy_move(sub, vec)

    fprint("Arrived to hydrophones! Going down!")
    yield sub.move.to_height(PINGER_HEIGHT).zero_roll_and_pitch().go(speed=0.1)


@util.cancellableInlineCallbacks
def fancy_move(sub, vec):
    global markers
    marker = Marker(
        ns="pinger",
        action=visualization_msgs.Marker.ADD,
        type=Marker.ARROW,
        scale=Vector3(0.2, 0.3, 0.1),
        points=np.array([Point(0, 0, 0), Point(vec[0], vec[1], vec[2])]),
    )
    marker.id = 4
    marker.header.frame_id = "/base_link"
    marker.color.r = 0
    marker.color.g = 0
    marker.color.b = 1
    marker.color.a = 1
    markers.markers.append(marker)

    yield sub.move.relative(vec).depth(MOVE_AT_DEPTH).zero_roll_and_pitch().go(
        speed=SPEED
    )
    yield sub.nh.sleep(3)


@util.cancellableInlineCallbacks
def go_to_random_guess(sub, pinger_1_req, pinger_2_req):
    pinger_guess = yield transform_to_baselink(sub, pinger_1_req, pinger_2_req)
    where_to = random.choice(pinger_guess)
    where_to = where_to / np.linalg.norm(where_to)
    fprint("Going to random guess {}".format(where_to), msg_color="yellow")
    yield fancy_move(sub, where_to)


def check_with_guess(vec, pinger_guess):
    for guess in pinger_guess:
        guess[2] = 0
    dots = [vec.dot(guess / np.linalg.norm(guess)) for guess in pinger_guess]
    fprint("Dots {}".format(dots))
    if dots[0] < 0.6 and dots[1] < 0.6:
        # Get the guess that is close to pnger vec
        go_to_guess = pinger_guess[np.argmax(dots)]
        go_to_guess = go_to_guess / np.linalg.norm(go_to_guess)
        fprint(
            "Though ping was behind. Going to pinger guess {} at {}".format(
                np.argmax(dots) + 1, go_to_guess
            ),
            msg_color="yellow",
        )
        return (False, go_to_guess)
    return (True, vec)


@util.cancellableInlineCallbacks
def transform_to_baselink(sub, pinger_1_req, pinger_2_req):
    transform = yield sub._tf_listener.get_transform("/base_link", "/map")
    pinger_guess = [
        transform._q_mat.dot(
            mil_ros_tools.rosmsg_to_numpy(x.location.pose.position) - transform._p
        )
        for x in (pinger_1_req, pinger_2_req)
    ]
    for idx, guess in enumerate(pinger_guess):
        marker = Marker(
            ns="pinger",
            action=visualization_msgs.Marker.ADD,
            type=Marker.ARROW,
            scale=Vector3(0.1, 0.2, 0),
            points=np.array([Point(0, 0, 0), Point(guess[0], guess[1], guess[2])]),
        )
        marker.id = idx
        marker.header.frame_id = "/base_link"
        marker.color.r = 0
        marker.color.g = 1
        marker.color.a = 1
        global markers
        markers.markers.append(marker)
    defer.returnValue(pinger_guess)
