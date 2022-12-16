from mil_misc_tools import text_effects

from .sub_singleton import SubjuGator

SPEED_LIMIT = 0.5  # m/s

fprint = text_effects.FprintFactory(title="STRIPPER", msg_color="cyan").fprint


class Strip(SubjuGator):
    async def pitch(self):
        start = self.move.forward(0).zero_roll_and_pitch()
        pitches = [start.pitch_down_deg(7), start] * 5
        for p in pitches:
            await p.go(speed=0.1)

    async def run(self, args):
        fprint("Starting...")
        sub_start_position, sub_start_orientation = await self.tx_pose()
        fprint(sub_start_orientation)
        await self.nh.sleep(2)

        sub_start = self.move.forward(0).zero_roll_and_pitch()
        await sub_start.down(0.15).set_orientation(sub_start_orientation).go(speed=0.1)
        await self.nh.sleep(3)

        start = sub_start.forward(0).zero_roll_and_pitch()
        # fprint('Searching... pitching...')
        # await pitch(sub)
        fprint("Moving to gate")
        gate = start.forward(3).down(0.2)
        await gate.go(speed=SPEED_LIMIT)
        await self.nh.sleep(5)
        # fprint('Searching... pitching')
        # await pitch(sub)
        fprint("Going right in front of pole")
        pole = gate.forward(8.6).down(0.5)
        await pole.go(speed=SPEED_LIMIT)
        await self.nh.sleep(5)

        fprint("Going around pole")
        offset_left = 1.5
        offset_forward = 2

        pole_l = pole.left(offset_left).set_orientation(sub_start_orientation)
        await pole_l.go(speed=SPEED_LIMIT)
        await self.nh.sleep(3)

        pole_f = pole_l.forward(offset_forward).yaw_right_deg(10)
        await pole_f.go(speed=SPEED_LIMIT)
        await self.nh.sleep(3)

        pole_r = self.move.right(offset_left * 2)
        await pole_r.go(speed=SPEED_LIMIT)
        await self.nh.sleep(3)

        pole_b = pole_r.backward(offset_forward + offset_forward)
        await pole_b.go(speed=SPEED_LIMIT)
        await self.nh.sleep(3)

        await self.move.strafe_left(offset_left).go(speed=SPEED_LIMIT)
        await self.nh.sleep(5)

        fprint("Turning back to gate")
        # await pole.backward(1).go(speed=SPEED_LIMIT)
        await self.nh.sleep(5)
        fprint("Look at gate")
        await self.move.look_at(gate._pose.position).yaw_left_deg(10).go(
            speed=SPEED_LIMIT
        )
        await self.nh.sleep(5)
        #        fprint('Going to gate')
        await self.move.forward(8.9).depth(0.6).go(speed=SPEED_LIMIT)


#        await start.go(speed=SPEED_LIMIT)
# await gate.yaw_left_deg(180).down(0.1).go(speed=SPEED_LIMIT)
#        await self.nh.sleep(5)
#        fprint('Go past through gate')
#        await start.yaw_left_deg(180).go(speed=SPEED_LIMIT)
