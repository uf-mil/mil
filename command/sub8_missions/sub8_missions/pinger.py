from txros import util
import rospy
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from mil_passive_sonar.msg import ProcessedPing
# from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from std_srvs.srv import Trigger
from twisted.internet import defer
import random
import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

ALPHA = 30000
BETA = 40000
CHARLIE = 25000
DELTA = 35000
SCHWARTZ = 37000

SPEED = 1
FREQUENCY = CHARLIE
FREQUENCY_TOL = 3000

PINGER_HEIGHT = 3 # how high to go above pinger after found
MOVE_AT_DEPTH = 1.6  # how low to swim and move

POSITION_TOL = 0.1 # how close to pinger before quiting
Z_POSITION_TOL = -0.53


class Pinger(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        global markers
        markers = MarkerArray()
        pub_markers = yield self.nh.advertise('/pinger/rays', MarkerArray)

        fprint('Getting Guess Locations')

        use_prediction = False

        try:
            pinger_1_req = yield self.poi.get('pinger_surface')
            pinger_2_req = yield self.poi.get('pinger_shooter')
            fprint('Found two pinger guesses {} {}'.format(pinger_1_req, pinger_2_req), msg_color='green')
            pinger_midpoint = (pinger_1_req + pinger_2_req) / 2
            half_way = pinger_midpoint / 2
            fprint('Looking at the poi')
            yield self.move.look_at_without_pitching(pinger_midpoint).go(speed=0.5)
            yield self.nh.sleep(3)
            fprint('Moving to the midpoint of the midpoint poi')
            yield self.move.set_position(half_way).go(speed=1)
            yield self.nh.sleep(5)


            pinger_guess = yield self.transform_to_baselink(
                self, pinger_1_req, pinger_2_req)
            fprint(pinger_guess)
        except Exception as e:
            fprint(
                'Failed to /guess_location. Procceding without guess',
                msg_color='yellow')
            use_prediction = False
            fprint(e)

        while True:
            fprint('=' * 50)

            fprint("waiting for message")
            p_message = yield self.pinger_sub.get_next_message()

            pub_markers.publish(markers)
            fprint(p_message)

            # Ignore freuqnecy
            if not abs(p_message.freq - FREQUENCY) < FREQUENCY_TOL:
                fprint(
                    "Ignored! Recieved Frequency {}".format(p_message.freq),
                    msg_color='red')
                if use_prediction:
                    fprint("Moving to random guess")
                    yield self.go_to_random_guess(self, pinger_1_req, pinger_2_req)
                continue

            # Ignore magnitude from processed ping
            p_position = mil_ros_tools.rosmsg_to_numpy(p_message.position)
            vec = p_position / np.linalg.norm(p_position)
            if np.isnan(vec).any():
                fprint('Ignored! nan', msg_color='red')
                if use_prediction:
                    yield self.go_to_random_guess(self, pinger_1_req, pinger_2_req)
                continue

            # Tranform hydrophones vector to relative coordinate
            transform = yield self._tf_listener.get_transform(
                '/base_link', '/hydrophones')
            vec = transform._q_mat.dot(vec)

            fprint('Transformed vec: {}'.format(vec))
            marker = Marker(
                ns='pinger',
                action=visualization_msgs.Marker.ADD,
                type=Marker.ARROW,
                scale=Vector3(0.2, 0.5, 0),
                points=np.array([Point(0, 0, 0),
                                 Point(vec[0], vec[1], vec[2])]))
            marker.id = 3
            marker.header.frame_id = '/base_link'
            marker.color.r = 1
            marker.color.g = 0
            marker.color.a = 1
            markers.markers.append(marker)

            # Check if we are on top of pinger
            if abs(vec[0]) < POSITION_TOL and abs(vec[1]) < POSITION_TOL and vec[2] < Z_POSITION_TOL:
                sub_position, _ = yield self.tx_pose()
                dists = [
                    np.linalg.norm(sub_position - x)
                    for x in (pinger_1_req, pinger_2_req)
                ]
                pinger_id = np.argmin(dists)
                # pinger_id 0 = pinger_surface
                # pinger_id 1 = pinger_shooter
                if pinger_id == 0:
                    # yikes, my bad guys
                    yield self.nh.set_param("octagon_position_x", str(self.pose.position[0]))
                    yield self.nh.set_param("octagon_position_y", str(self.pose.position[1]))
                    yield self.nh.set_param("octagon_position_z", str(self.pose.position[2]))
                    fprint('Set {}'.format(self.pose.position))
          
                if pinger_id == 1:
                    fprint('===DOING SHOOTER SETUP===', msg_color='green')
                    sub_pos = yield self.pose.position
                    sub_vec = -sub_pos / np.linalg.norm(sub_pos)
                    fprint('Moving away'.format(sub_vec))
                    yield self.move.set_position(sub_pos + ((2.5 * sub_vec)).depth(MOVE_AT_DEPTH).go(speed=SPEED)
                    yield self.nh.sleep(1)
                    fprint('Looking at shooter')
                    yield self.move.look_at_without_pitching(sub_pos).go()
                yield self.nh.set_param("pinger_where", int(pinger_id))

                break

            vec[2] = 0
            if use_prediction:
                pinger_guess = yield self.transform_to_baselink(
                    self, pinger_1_req,
                                                           pinger_2_req)
                fprint('Transformed guess: {}'.format(pinger_guess))
                # Check if the pinger aligns with guess
                # check, vec = self.check_with_guess(vec, pinger_guess)

            fprint('move to {}'.format(vec))
            yield self.fancy_move(self, vec)

        fprint('Arrived to hydrophones! Going down!')
        yield self.move.to_height(PINGER_HEIGHT).zero_roll_and_pitch().go(speed=0.1)

    @util.cancellableInlineCallbacks
    def fancy_move(self, sub, vec):
        global markers
        marker = Marker(
            ns='pinger',
            action=visualization_msgs.Marker.ADD,
            type=Marker.ARROW,
            scale=Vector3(0.2, 0.3, 0.1),
            points=np.array([Point(0, 0, 0),
                             Point(vec[0], vec[1], vec[2])]))
        marker.id = 4
        marker.header.frame_id = '/base_link'
        marker.color.r = 0
        marker.color.g = 0
        marker.color.b = 1
        marker.color.a = 1
        markers.markers.append(marker)

        yield sub.move.relative(vec).depth(MOVE_AT_DEPTH).zero_roll_and_pitch().go(
            speed=SPEED)

    @util.cancellableInlineCallbacks
    def go_to_random_guess(self, sub, pinger_1_req, pinger_2_req):
        pinger_guess = yield self.transform_to_baselink(sub, pinger_1_req, pinger_2_req)
        where_to = random.choice(pinger_guess)
        where_to = where_to / np.linalg.norm(where_to)
        fprint('Going to random guess {}'.format(where_to), msg_color='yellow')
        yield self.fancy_move(sub, where_to)

    def check_with_guess(self, vec, pinger_guess):
        for guess in pinger_guess:
            guess[2] = 0
        dots = [vec.dot(guess / np.linalg.norm(guess))
                for guess in pinger_guess]
        fprint('Dots {}'.format(dots))
        if dots[0] < 0.6 and dots[1] < 0.6:
            # Get the guess that is close to pnger vec
            go_to_guess = pinger_guess[np.argmax(dots)]
            go_to_guess = go_to_guess / np.linalg.norm(go_to_guess)
            fprint(
                'Though ping was behind. Going to pinger guess {} at {}'.format(
                    np.argmax(dots) + 1, go_to_guess),
                msg_color='yellow')
            return (False, go_to_guess)
        return (True, vec)

    @util.cancellableInlineCallbacks
    def transform_to_baselink(self, sub, pinger_1_req, pinger_2_req):
        transform = yield sub._tf_listener.get_transform('/base_link', '/map')
        position = yield sub.pose.position
        pinger_guess = [
            transform._q_mat.dot(np.array(x) - position)\
                for x in (pinger_1_req, pinger_2_req)
        ]
        for idx, guess in enumerate(pinger_guess):
            marker = Marker(
                ns='pinger',
                action=visualization_msgs.Marker.ADD,
                type=Marker.ARROW,
                scale=Vector3(0.1, 0.2, 0),
                points=np.array(
                    [Point(0, 0, 0),
                     Point(guess[0], guess[1], guess[2])]))
            marker.id = idx
            marker.header.frame_id = '/base_link'
            marker.color.r = 0
            marker.color.g = 1
            marker.color.a = 1
            global markers
            markers.markers.append(marker)
        defer.returnValue(pinger_guess)
