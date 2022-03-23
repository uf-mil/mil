from txros import util
import rospy
import numpy as np
import mil_ros_tools
from mil_misc_tools import text_effects
from mil_passive_sonar.msg import ProcessedPing
# from sub8_msgs.srv import GuessRequest, GuessRequestRequest
from std_srvs.srv import Trigger
from mil_tools import rosmsg_to_numpy
import tf2_ros
from twisted.internet import defer
import random
import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from .sub_singleton import SubjuGator

fprint = text_effects.FprintFactory(title="PINGER", msg_color="cyan").fprint

SPEED = 0.75
FREQUENCY = 37000
FREQUENCY_TOL = 3000

PINGER_HEIGHT = 0.75  # how high to go above pinger after found
MOVE_AT_DEPTH = 0.5  # how low to swim and move

POSITION_TOL = 0.09  # how close to pinger before quiting
Z_POSITION_TOL = -0.53


class Pinger2(SubjuGator):

    @util.cancellableInlineCallbacks
    def run(self, args):
        global markers
        markers = MarkerArray()
        pub_markers = yield self.nh.advertise('/pinger/rays', MarkerArray)

        fprint('Getting Guess Locations')

        pass_scan = True
        if pass_scan:
            yield self.run_pass_scan(args)
        else:
            yield self.run_dual_scan(args)

        use_prediction = False

        try:
            save_pois = rospy.ServiceProxy(
                '/poi_server/save_to_param', Trigger)
            _ = save_pois()
            if rospy.has_param('/poi_server/initial_pois/pinger_shooter') and\
               rospy.has_param('/poi_server/initial_pois/pinger_surface'):
                fprint('Found two pinger guesses', msg_color='green')
                pinger_1_req = rospy.get_param('/poi_server/initial_pois/pinger_surface')
                pinger_2_req = rospy.get_param('/poi_server/initial_pois/pinger_shooter')

                # check \/
                pinger_guess = yield self.transform_to_baselink(
                    self, pinger_1_req, pinger_2_req)
                fprint(pinger_guess)
            else:
                use_prediction = False
                fprint('Forgot to add pinger to guess server?',
                        msg_color='yellow')
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
                if not use_prediction:
                    fprint("Arrived to pinger!")
                    break

                sub_position, _ = yield self.tx_pose()
                dists = [
                    np.linalg.norm(sub_position - mil_ros_tools.rosmsg_to_numpy(
                        x.location.pose.position))
                    for x in (pinger_1_req, pinger_2_req)
                ]
                pinger_id = np.argmin(dists)
                # pinger_id 0 = pinger_surface
                # pinger_id 1 = pinger_shooter
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

    ###############################################################################################
    @util.cancellableInlineCallbacks
    def run_dual_scan(self, args):
        # Parse Parameters
        uses_multilateration = args.multilateration
        scan_dist = args.scandist
        should_kill = args.nokill
        listen_time = args.scantime

        # Calculate scan points
        scan_points_0 = yield self.get_perpendicular_points(self.gate_totems[1], scan_dist)
        scan_points_1 = yield self.get_perpendicular_points(self.gate_totems[2], scan_dist)
        scan_point_0 = scan_points_0[0]
        scan_lookat_0 = scan_points_0[1]
        scan_point_1 = scan_points_1[0]
        scan_lookat_1 = scan_points_1[1]

        if uses_multilateration:
            # Reset Scan
            yield self.hydrophones.disable()
            yield self.hydrophones.reset()
        else:
            # Reset pings
            self.intersect_vectors = []

        # Execute Scan 1

        self.send_feedback('Navigating to scan start point 1')
        yield self.move.set_position(scan_point_0).look_at(scan_lookat_0).go()

        # If kill is enabled, kill the thrusters
        if should_kill:
            self.disable_thrusters()

        if uses_multilateration:
            # Turn on multilateration
            self.hydrophones.enable()
        else:
            # Enable ping counting
            self.hydrophones.set_callback(self.ping_recv)

        # Sleep
        yield self.nh.sleep(listen_time)

        if uses_multilateration:
            # Disable multilateration
            self.hydrophones.disable()
        else:
            # Disable ping counting
            self.hydrophones.set_callback(None)

        # If kill is enabled, revive the thrusters
        if should_kill:
            self.enable_autonomous()

        # Execute Scan 2

        self.send_feedback('Navigating to scan start point 2')
        yield self.move.set_position(scan_point_1).look_at(scan_lookat_1).go()

        # If kill is enabled, kill the thrusters
        if should_kill:
            self.disable_thrusters()

        if uses_multilateration:
            # Turn on multilateration
            self.hydrophones.enable()
        else:
            # Enable ping counting
            self.hydrophones.set_callback(self.ping_recv)

        # Sleep
        yield self.nh.sleep(listen_time)

        if uses_multilateration:
            # Disable multilateration
            self.hydrophones.disable()
        else:
            # Disable ping counting
            self.hydrophones.set_callback(None)

        # If kill is enabled, revive the thrusters
        if should_kill:
            self.enable_autonomous()

        # Calculate results
        if uses_multilateration:
            multilateration_results = yield self.get_multilateration_closest()
            self.pinger_gate = multilateration_results[0]
            multilateration_confidence = multilateration_results[1]

            self.send_feedback('Multilaterating Method has confidence of ' + str(multilateration_confidence))
        else:
            intersect_results = self.calculate_intersect()
            self.pinger_gate = intersect_results[0]
            intersect_confidence = intersect_results[1]

            self.send_feedback('Intersecting Method has confidence of ' + str(intersect_confidence))

    @util.cancellableInlineCallbacks
    def run_pass_scan(self, args):
        # Parse Parameters
        uses_multilateration = args.multilateration
        scan_dist = args.scandist
        speed = args.speed

        # Calculate scan points
        scan_points_0 = yield self.get_perpendicular_points(self.gate_centers[0], scan_dist)
        scan_points_1 = yield self.get_perpendicular_points(self.gate_centers[2], scan_dist)
        scan_points_2 = yield self.get_perpendicular_points(self.gate_totems[3], scan_dist)
        scan_point_0 = scan_points_0[0]
        scan_lookat_0 = scan_points_1[0]
        scan_point_1 = scan_points_1[0]
        scan_lookat_1 = scan_points_2[0]

        if uses_multilateration:
            # Reset Scan
            yield self.hydrophones.disable()
            yield self.hydrophones.reset()
        else:
            # Reset pings
            self.intersect_vectors = []

        # Execute Scan

        self.send_feedback('Navigating to scan start point')
        yield self.move.set_position(scan_point_0).look_at(scan_lookat_0).go()

        if uses_multilateration:
            # Turn on multilateration
            self.hydrophones.enable()
        else:
            # Enable ping counting
            self.hydrophones.set_callback(self.ping_recv)

        self.send_feedback('Navigating to scan end point')
        yield self.move.set_position(scan_point_1).look_at(scan_lookat_1).go(speed_factor=speed)

        if uses_multilateration:
            # Disable multilateration
            self.hydrophones.disable()
        else:
            # Disable ping counting
            self.hydrophones.set_callback(None)

        # Calculate results
        if uses_multilateration:
            multilateration_results = yield self.get_multilateration_closest()
            self.pinger_gate = multilateration_results[0]
            multilateration_confidence = multilateration_results[1]

            self.send_feedback('Multilaterating Method has confidence of ' + str(multilateration_confidence))
        else:
            intersect_results = self.calculate_intersect()
            self.pinger_gate = intersect_results[0]
            intersect_confidence = intersect_results[1]

            self.send_feedback('Intersecting Method has confidence of ' + str(intersect_confidence))

    @util.cancellableInlineCallbacks
    def get_multilateration_closest(self):
        # Get the most recent multilateration position
        rospos = yield self.hydrophones.get_last_position()
        pinger_pos = rosmsg_to_numpy(rospos.point)[0:2]

        # Calculate the distances to each gate center
        distances = []
        for gate_center in self.gate_centers:
            distances.append((gate_center[0] - pinger_pos[0]) ** 2 + (gate_center[1] - pinger_pos[1]) ** 2)

        # Determine which gate is closest
        pinger_gate = np.argmin(np.array(distances))

        # Calculate the confidence
        confidence = (5 - distances[pinger_gate]) / 5.0
        defer.returnValue((pinger_gate, confidence))

    @util.cancellableInlineCallbacks
    def ping_recv(self, p_message):
        try:
            # Transform the ping into enu
            hydrophones_to_enu = yield self.tf_listener.get_transform('enu', p_message.header.frame_id)
            hydrophones_origin = hydrophones_to_enu._p[0:2]
            heading = rosmsg_to_numpy(p_message.vector)
            heading_enu = hydrophones_to_enu.transform_vector(heading)
            heading_enu = heading_enu[0:2] / np.linalg.norm(heading_enu[0:2])

            # Track the ping
            self.intersect_vectors.append((hydrophones_origin[0:2], hydrophones_origin[0:2] + heading_enu[0:2]))
        except tf2_ros.TransformException, e:
            self.send_feedback('TF Exception: {}'.format(e))

    def calculate_intersect(self):
        gate_vector = np.array([self.gate_totems[0], self.gate_totems[3]])
        bucket = [0, 0, 0]
        gateDist = [0, 0, 0]
        for pinger_vector in self.intersect_vectors:
            # Calculate intersection between the gate and the pinger vector
            t, s = np.linalg.solve(np.array([gate_vector[1] - gate_vector[0], pinger_vector[0] - pinger_vector[1]]).T,
                                   pinger_vector[0] - gate_vector[0])
            intersection = (1 - t) * gate_vector[0] + t * gate_vector[1]

            # Calculate the distances from each gate center to the intersection
            gateDist[0] = np.linalg.norm(intersection - np.array(self.gate_centers[0]))
            gateDist[1] = np.linalg.norm(intersection - np.array(self.gate_centers[1]))
            gateDist[2] = np.linalg.norm(intersection - np.array(self.gate_centers[2]))

            # If the intersection is more than 15 meters from the center of the gates it is outside the gates
            if gateDist[1] > 16:
                # Ignore it
                continue

            # Calculate vectors to the edges of the gates and the intersect
            vec_corner1 = gate_vector[0] - pinger_vector[0]
            vec_corner1 = vec_corner1 / np.linalg.norm(vec_corner1)
            vec_corner4 = gate_vector[1] - pinger_vector[0]
            vec_corner4 = vec_corner4 / np.linalg.norm(vec_corner4)
            vec_intersect = intersection - pinger_vector[0]
            vec_intersect = vec_intersect / np.linalg.norm(vec_intersect)

            # Calculate the angle between the edges of the gates and the angle between an edge and the intersection
            ang_c1_c4 = np.arccos(np.clip(np.dot(vec_corner1, vec_corner4), -1.0, 1.0))
            ang_c1_i4 = np.arccos(np.clip(np.dot(vec_corner1, vec_corner4), -1.0, 1.0))

            # If the angles are out of range, throw out this intersection
            if ang_c1_c4 >= 0 and (ang_c1_i4 < 0 or ang_c1_i4 > ang_c1_c4):
                continue
            if ang_c1_c4 < 0 and (ang_c1_i4 > 0 or ang_c1_i4 < ang_c1_c4):
                continue

            # Determine which gate the intersection is in
            bucket_index = np.argmin(np.array(gateDist))
            bucket[bucket_index] = bucket[bucket_index] + 1

        # Determine which gate has the most hits
        pinger_gate = np.argmax(np.array(bucket))

        # Calculate the confidence
        confidence = bucket[pinger_gate] / (bucket[0] + bucket[1] + bucket[2])
        return (pinger_gate, confidence)

