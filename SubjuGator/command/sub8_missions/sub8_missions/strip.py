from txros import util
from mil_misc_tools import text_effects
from .sub_singleton import SubjuGator

SPEED_LIMIT = 0.15  # m/s

fprint = text_effects.FprintFactory(title="STRIPPER", msg_color="cyan").fprint


class Strip(SubjuGator):
    @util.cancellableInlineCallbacks
    def pitch(self):
        start = self.move.forward(0).zero_roll_and_pitch()
        pitches = [start.pitch_down_deg(7), start] * 5
        for p in pitches:
            yield p.go(speed=0.1)
    
    
    @util.cancellableInlineCallbacks
    def run(self, args):
        fprint('Starting...')
        sub_start_position, sub_start_orientation = yield self.tx_pose()
        fprint(sub_start_orientation)
        yield self.nh.sleep(2)
    
        sub_start = self.move.forward(0).zero_roll_and_pitch()
        yield sub_start.down(0.3).set_orientation(sub_start_orientation).go(
            speed=0.1)
        yield self.nh.sleep(3)
    
        start = sub_start.forward(0).zero_roll_and_pitch()
        # fprint('Searching... pitching...')
        # yield pitch(sub)
        fprint('Moving to gate')
        gate = start.forward(3).down(0.3)
        yield gate.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(5)
        # fprint('Searching... pitching')
        # yield pitch(sub)
        fprint('Going right in front of pole')
        pole = gate.forward(8.7).down(0.5)
        yield pole.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(5)
    
        fprint('Going around pole')
        offset_left = 1.5
        offset_forward = 1.8
    
        pole_l = pole.left(offset_left)
        yield pole_l.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(3)
    
        pole_f = pole_l.forward(offset_forward)
        yield pole_f.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(3)
    
        pole_r = pole_f.right(offset_left * 2 - offset_left / 2)
        yield pole_r.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(3)
    
        pole_b = pole_r.backward(offset_forward * 2 - offset_forward / 2)
        yield pole_b.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(3)
    
        yield pole.go(speed=SPEED_LIMIT)
        yield self.nh.sleep(5)
    
        fprint('Turning back to gate')
        yield pole.backward(1).go(speed=SPEED_LIMIT)
        yield self.nh.sleep(5)
        fprint('Look at gate')
        yield self.move.look_at(gate._pose.position).go(speed=SPEED_LIMIT)
        yield self.nh.sleep(5)
        fprint('Going to gate')
        yield gate.yaw_left_deg(180).down(0.1).go(speed=SPEED_LIMIT)
        yield self.nh.sleep(5)
        fprint('Go past through gate')
        yield start.yaw_left_deg(180).go(speed=SPEED_LIMIT)
