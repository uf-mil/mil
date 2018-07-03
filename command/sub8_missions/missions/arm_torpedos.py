from txros import util
import numpy as np
from mil_ros_tools import rosmsg_to_numpy
from mil_misc_tools import FprintFactory

MISSION = 'Torpedo Challenge'


class Target(object):
    def __init__(self):
        self.position = None
        self.destroyed = False

    def set_destroyed(self):
        self.destroyed = True

    def update_position(self, pos):
        self.position = pos


class FireTorpedos(object):
    '''
    Mission to solve the torpedo RoboSub challenge.

    This code was based off of the Buoy mission code written by Kevin Allen.
    Its goal is to search for a target on the torpedo board and fire at it.
    '''
    TIMEOUT_SECONDS = 30
    Z_PATTERN_RADIUS = 0.3
    Y_PATTERN_RADIUS = 2.0
    BACKUP_METERS = 3.0
    BLIND = True

    def __init__(self, sub):
        self.sub = sub
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(
            title=MISSION, msg_color="green").fprint
        # B = bottom; T = Top; L = left; R = right; C = center; O = unblocked;
        # X = blocked;
        self.targets = {
            'TCX': Target(),
            'TRX': Target(),
            'TLX': Target(),
            'BCO': Target()
        }
        self.pattern_done = False
        self.done = False
        self.generate_pattern()

    def generate_pattern(self):
        z = self.Z_PATTERN_RADIUS
        y = self.Y_PATTERN_RADIUS
        self.moves = [[0, 0, z], [0, y, 0], [0, 0, -2 * z], [0, -2 * y, 0]]
        self.move_index = 0

    @util.cancellableInlineCallbacks
    def search(self):
        while True:
            info = 'CURRENT TARGETS: '

            target = 'BCO'
            '''
            In the event we want to attempt other targets beyond bare minimum
            for target in self.targets. Eventually we will want to take the
            best target, which is the TCX target. Priority targetting will
            come once we confirm we can actually unblock currently blocked
            targets.
            '''
            res = yield self.sub.vision_proxies.arm_torpedos.get_pose(
                target='board')
            if res.found:
                self.targets[target].update_position(
                    rosmsg_to_numpy(res.pose.pose.position))
            if self.targets[target].position is not None:
                info += target + ' '
            yield self.sub.nh.sleep(0.5)  # Throttle service calls
            self.print_info(info)

    @util.cancellableInlineCallbacks
    def pattern(self):
        def err():
            self.print_info('Search pattern canceled')

        self.pattern_done = False
        for i, move in enumerate(self.moves[self.move_index:]):
            move = self.sub.move.relative(np.array(move)).go(blind=self.BLIND)
            move.addErrback(err)
            yield move
            self.move_index = i + 1
        self.print_bad('Pattern finished. Firing at any locked targets.')
        self.pattern_done = True

    @util.cancellableInlineCallbacks
    def fire(self, target):
        self.print_info("FIRING {}".format(target))
        yield self.sub.move.go(blind=self.BLIND)  # Station hold
        target_position = self.targets[target].position
        yield self.sub.move.depth(-target_position[2]).go(blind=self.BLIND)
        yield self.sub.move.look_at_without_pitching(target_position).go(
            blind=self.BLIND)
        yield self.sub.move.set_position(target_position).backward(1).go(
            blind=self.BLIND)
        self.print_good(
            "{} locked. Firing torpedos. Hit confirmed, good job Commander.".
            format(target))
        self.done = True

    def get_target(self):
        target = 'BCO'
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
        self.sub.vision_proxies.arm_torpedos.start()

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
        self.print_good('Done!')


@util.cancellableInlineCallbacks
def run(sub):
    # print('running')
    mission = FireTorpedos(sub)
    yield mission.run()
