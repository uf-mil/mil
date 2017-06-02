from txros import util
import numpy as np
from mil_ros_tools import rosmsg_to_numpy
from mil_misc_tools import FprintFactory

MISSION = "BUMP BUOYS"


class Buoy(object):
    def __init__(self):
        self.position = None
        self.bumped = False

    def set_bumped(self):
        self.bumped = True

    def update_position(self, pos):
        self.position = pos


class BumpBuoysMission(object):
    '''
    Mission to solve the recurring bump buoys RoboSub challenge.

    Designed to use the async features of txros to solve the mission in as little
    time as possible. One async function constantly pings the percption node for
    the latest pose, if available, for each color buoy. Another function continually
    checks if a new buoy has been found and performs moves to bump it. Another function
    runs search patterns (left, right, up, down) to attempt to gain more observations on
    buoys between bumping moves.
    '''
    ORDER = ['red', 'green', 'yellow']
    TIMEOUT_SECONDS = 30

    def __init__(self, sub):
        self.sub = sub
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(title=MISSION, msg_color="green").fprint
        self.buoys = {'red': Buoy(), 'green': Buoy(), 'yellow': Buoy()}
        self.pattern_done = False
        self.generate_pattern()

    def generate_pattern(self):
        self.moves = [[0, 0, 0.3],
                      [0, 1.5, 0],
                      [0, 0, -2 * 0.3],
                      [0, -2 * 1.5, 0]]
        self.move_index = 0

    @util.cancellableInlineCallbacks
    def search(self):
        while True:
            info = 'FOUND: '
            for buoy in self.buoys:
                res = yield self.sub.vision_proxies.buoy.get_pose(target=buoy)
                if res.found:
                    self.buoys[buoy].update_position(rosmsg_to_numpy(res.pose.pose.position))
                if self.buoys[buoy].position is not None:
                    info += buoy + ' '
                yield self.sub.nh.sleep(0.5)  # Throttle service calls
            self.print_info(info)

    @util.cancellableInlineCallbacks
    def pattern(self):

        def err():
            self.print_info('Search pattern canceled')

        self.pattern_done = False
        for i, move in enumerate(self.moves[self.move_index:]):
            move = self.sub.move.relative(np.array(move)).go()
            move.addErrback(err)
            yield move
            self.move_index = i + 1
        self.print_bad('Pattern finished. Bumping any identifed buoys')
        self.pattern_done = True

    @util.cancellableInlineCallbacks
    def bump(self, buoy):
        self.print_info("BUMPING {}".format(buoy))
        yield self.sub.move.go()  # Station hold
        buoy_position = self.buoys[buoy].position
        yield self.sub.move.depth(-buoy_position[2]).go()
        yield self.sub.move.look_at_without_pitching(buoy_position).go()
        yield self.sub.move.set_position(buoy_position).forward(0.2).go()
        self.print_good("{} BUMPED. Backing up".format(buoy))
        yield self.sub.move.backward(3.5).go()

    def get_next_bump(self):
        '''
        Returns the color of the buoy that should be bumped now, or None.
        Tries to bump in the desired order, but if the search pattern is exhasted
        just goes for whichever have been found!
        '''
        for color in self.ORDER:
            if self.buoys[color].bumped:
                continue
            elif self.buoys[color].position is not None:
                return color
            elif self.pattern_done:
                return color
            else:
                return None

    def done(self):
        bumped = sum(map(lambda b: 1 if self.buoys[b].bumped else 0, self.buoys))
        return bumped == len(self.ORDER)

    @util.cancellableInlineCallbacks
    def run(self):
        # start_time = yield self.sub.nh.get_time()  # Store time mission starts for timeout

        self.print_info("Enabling Perception")
        self.sub.vision_proxies.buoy.start()

        pattern = self.pattern()
        self.do_search = True
        search = self.search()
        while not self.done():
            b = self.get_next_bump()
            if b is not None:
                pattern.cancel()
                yield self.bump(b)
                self.buoys[b].set_bumped()
                if not self.done():
                    pattern = self.pattern()
            yield self.sub.nh.sleep(0.1)
        search.cancel()
        pattern.cancel()
        self.print_good('Done!')


@util.cancellableInlineCallbacks
def run(sub):
    mission = BumpBuoysMission(sub)
    yield mission.run()
