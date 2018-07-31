from txros import util
import tf
import rospy
import numpy as np
from mil_ros_tools import rosmsg_to_numpy
import visualization_msgs.msg as visualization_msgs
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Vector3
from mil_misc_tools import FprintFactory

MISSION = 'Roulette Challenge'


class Target(object):

    def __init__(self):
        self.position = None
        self.destroyed = False

    def set_destroyed(self):
        self.destroyed = True

    def update_position(self, pos):
        self.position = pos


class DropTheBall(object):
    '''
    Mission to solve the torpedo RoboSub challenge.

    This code was based off of the Buoy mission code written by Kevin Allen.
    Its goal is to search for a target on the torpedo board and fire at it.
    '''
    TIMEOUT_SECONDS = 30
    X_PATTERN_RADIUS = 1.0
    Y_PATTERN_RADIUS = 1.0
    BACKUP_METERS = 3.0
    BLIND = True

    def __init__(self, sub):
        self.sub = sub
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(
            title=MISSION, msg_color="green").fprint
        self.tf_listener = tf.TransformListener()

        # B = bottom; T = Top; L = left; R = right; C = center; O = unblocked;
        # X = blocked;
        self.targets = {
            'Wheel': Target()
        }
        self.pattern_done = False
        self.done = False
        self.ltime = None
        self.generate_pattern()

    def generate_pattern(self):
        z = self.X_PATTERN_RADIUS
        y = self.Y_PATTERN_RADIUS
        self.moves = [[0, s * y, 0], [s * x, 0, 0], [0, s * -2 * y, 0],
                      [-2 * x * s, 0, 0], [0, s * y, 0], [s * x, 0, 0]]
        self.move_index = 0

    @util.cancellableInlineCallbacks
    def search(self):
        global markers
        markers = MarkerArray()
        pub_markers = yield self.sub.nh.advertise('/roulette_wheel/rays', MarkerArray)
        while True:
            info = 'CURRENT TARGETS: '

            target = 'Wheel'
            pub_markers.publish(markers)
            '''
            In the event we want to attempt other targets beyond bare minimum
            for target in self.targets. Eventually we will want to take the
            best target, which is the TCX target. Priority targetting will
            come once we confirm we can actually unblock currently blocked
            targets.
            '''
            res = yield self.sub.vision_proxies.arm_torpedos.get_pose(
                target='roulette_wheel')
            if res.found:
                self.ltime = res.pose.header.stamp
                self.targets[target].update_position(
                    rosmsg_to_numpy(res.pose.pose.position))
                marker = Marker(
                    ns='roulette_wheel',
                    action=visualization_msgs.Marker.ADD,
                    type=Marker.ARROW,
                    scale=Vector3(0.2, 0.5, 0),
                    points=np.array([Point(0, 0, 0),
                                     res.pose.pose.position]))
                marker.id = 3
                marker.header.frame_id = '/base_link'
                marker.color.r = 1
                marker.color.g = 0
                marker.color.a = 1
                markers.markers.append(marker)
            if self.targets[target].position is not None:
                info += target + ' '
            yield self.sub.nh.sleep(0.5)  # Throttle service calls
            self.print_info(info)

    @util.cancellableInlineCallbacks
    def pattern(self):        
    	
    	self.print_info('Descending to Depth...')
        yield self.sub.move.depth(1.5).go(blind=self.BLIND, speed=0.1)

        def err():
            self.print_info('Search pattern canceled')

        self.pattern_done = False
        for i, move in enumerate(self.moves[self.move_index:]):
            move = self.sub.move.relative(np.array(move)).go(blind=self.BLIND, speed=0.1)
            move.addErrback(err)
            yield move
            self.move_index = i + 1
        self.print_bad('Pattern finished. Firing at any locked targets.')
        self.pattern_done = True

    @util.cancellableInlineCallbacks
    def fire(self, target):
        self.print_info("DROPPING {}".format(target))
        target_pose = self.targets[target].position
        yield self.sub.move.go(blind=self.BLIND, speed=0.1)  # Station hold
        transform = yield self.sub._tf_listener.get_transform('/map', '/base_link')
        target_position = transform._q_mat.dot(
                target_pose - transform._p)

        sub_pos = yield self.sub.tx_pose()
        # print('Current Sub Position: ', sub_pos)

        # sub_pos = transform._q_mat.dot(
                # (sub_pos[0]) - transform._p)
        # target_position = target_position - sub_pos[0]
        # yield self.sub.move.look_at_without_pitching(target_position).go(
            # blind=self.BLIND, speed=.1)
        # print('Map Position: ', target_position)
        yield self.sub.move.relative(np.array([0, target_position[1], 0])).go(blind=True, speed=.1)
        yield self.sub.move.relative(np.array([target_position[0], 0, 0])).go(
            blind=self.BLIND, speed=.1)

        self.print_good(
            "{} locked. Dropping the ball... Ball dropped. We tried.".
            format(target))
        sub_pos = yield self.sub.tx_pose()
        print('Current Sub Position: ', sub_pos)
        yield self.sub.actuators.drop_marker()
        self.done = True

    def get_target(self):
        target = 'Wheel'
        '''
        Returns the target we are going to focus on. Loop through priorities.
        Highest priority is the TCX target, followed by the TRX and TLX.
        Lowest priority is the BCO target. Targets are ordered in the list
        already, so this will take the first target it can actually find.
        Currently this will always by the BCO target. This is because we don't
        differentiate between targets currently as we cannot unblock the
        blocked ones. This means there is only one target for us to lock onto.
        '''
        if self.targets[target].destroyed:
            pass
            # temp = target
        elif self.targets[target].position is not None:
            return target
        elif self.pattern_done and self.targets[target].position is not None:
            return target
        else:
            return None

    @util.cancellableInlineCallbacks
    def run(self):
        # start_time = yield self.sub.nh.get_time()  # Store time mission
        # starts for timeout

        self.print_info("Enabling Perception")
        self.sub.vision_proxies.roulette_wheel.start()

        pattern = self.pattern()
        self.do_search = True
        search = self.search()
        while not self.done:
            t = self.get_target()
            if t is not None:
                pattern.cancel()
                yield self.fire(t)
                self.targets[t].set_destroyed()
                if not self.done:
                    pattern = self.pattern()
            elif self.pattern_done:
                break
            yield self.sub.nh.sleep(0.1)
        search.cancel()
        pattern.cancel()
        self.sub.vision_proxies.roulette_wheel.stop()
        self.print_good('Done!')


@util.cancellableInlineCallbacks
def run(sub):
    # print('running')
    rospy.init_node('roulette_wheel', anonymous=False)
    mission = DropTheBall(sub)
    yield mission.run()
