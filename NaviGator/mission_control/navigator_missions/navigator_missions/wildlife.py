#!/usr/bin/env python3
from std_srvs.srv import SetBoolRequest

from .navigator import NaviGatorMission


class Wildlife(NaviGatorMission):
    async def run(self, parameters):
        # Go to autonomous mode
        await self.set_classifier_enabled.wait_for_service()
        await self.set_classifier_enabled(SetBoolRequest(data=True))
        await self.nh.sleep(4)
        await self.change_wrench("autonomous")

        try:
            t1 = await self.get_sorted_objects("red_cylinder", n=1)
            t1 = t1[1][0]
        except Exception as e:
            print("could not find stc_platform")
            # get all pcodar objects
            try:
                print("check for any objects")
                t1 = await self.get_sorted_objects(name="UNKNOWN", n=-1)
                t1 = t1[1][0]
            # if no pcodar objects, drive forward
            except Exception as e:
                print("literally no objects?")
                await self.move.forward(10).go()
                # get first pcodar objects
                t1 = await self.get_sorted_objects(name="UNKNOWN", n=-1)
                t1 = t1[1][0]
                # if still no pcodar objects, guess RGB and exit mission
            # go to nearest obj to get better data on that obj

            print("going to nearest small object")

        points = await self.move.d_spiral_point(t1, 5, 4, 1, "ccw", theta_offset=-1.57)
