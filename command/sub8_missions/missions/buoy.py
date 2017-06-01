from twisted.internet import defer
from txros import util, tf
import numpy as np
from std_srvs.srv import SetBool, SetBoolRequest
from mil_ros_tools import rosmsg_to_numpy
from mil_misc_tools import FprintFactory

MISSION = "BUMP BUOYS"


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
    def __init__(self, sub):
        self.sub = sub
        self.print_info = FprintFactory(title=MISSION).fprint
        self.print_bad = FprintFactory(title=MISSION, msg_color="red").fprint
        self.print_good = FprintFactory(title=MISSION, msg_color="green").fprint
        self.found = {'red': None, 'green': None, 'yellow': None}
        self.bumped = []
        self.do_search = False
        self.generate_pattern()

    def generate_pattern(self):
        self.moves = [[0, 0, 0.3],
                      [0, 1.5, 0],
                      [0, 0, -2*0.3],
                      [0, -2*1.5, 0]]
        self.move_index = 0
        
    @util.cancellableInlineCallbacks
    def search(self):
        while self.search:
            info = 'FOUND: '
            for color in self.found:
                res = yield self.sub.vision_proxies.buoy.get_pose(target=color)
                if res.found:
                    self.found[color] = rosmsg_to_numpy(res.pose.pose.position)
                if self.found[color] is not None:
                    info += color + ' '
                yield self.sub.nh.sleep(0.5) # Throttle service calls
            self.print_info(info)

    @util.cancellableInlineCallbacks
    def pattern(self):
        def err():
            print_info('Search pattern canceled')
        for i, move in enumerate(self.moves[self.move_index:]):
            move = self.sub.move.relative(np.array(move)).go()
            move.addErrback(err)
            yield move
            self.move_index = i + 1

    @util.cancellableInlineCallbacks
    def bump(self, buoy):
        self.print_info("BUMPING {}".format(buoy))
        yield self.sub.move.go() # Station hold
        start_pose = self.sub.pose
 
        buoy_position = self.found[buoy]
        yield self.sub.move.depth(-buoy_position[2]).go()
        yield self.sub.move.look_at_without_pitching(buoy_position).go()
        dist = np.linalg.norm(buoy_position - self.sub.pose.position)
        yield self.sub.move.set_position(buoy_position).forward(0.2).go()
        self.print_good("{} BUMPED. Backing up".format(buoy))
        yield self.sub.move.backward(3.5).go()

    @util.cancellableInlineCallbacks
    def run(self):
        self.print_info("Enabling Perception")
        self.sub.vision_proxies.buoy.start()

        pattern = self.pattern()
        self.do_search = True
        search = self.search()
        while len(self.bumped) != len(self.found):
            for color in self.found:
                if self.found[color] is not None and color not in self.bumped:
                    pattern.cancel()
                    yield self.bump(color)
                    self.bumped.append(color)
                    if len(self.bumped) != len(self.found):
                        pattern = self.pattern()
            yield self.sub.nh.sleep(0.1)
        search.cancel()
        self.print_good('Done!')

@util.cancellableInlineCallbacks
def run(sub):
    mission = BumpBuoysMission(sub)
    yield mission.run()
