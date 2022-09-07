from .base_mission import ExampleBaseMission


class SuperMission(ExampleBaseMission):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)

    @classmethod
    def init(cls):
        print("SuperMission init")

    async def run(self, parameters):
        self.send_feedback("Going to run two missions")
        await self.nh.sleep(0.5)
        self.send_feedback("Running PrintAndWait")
        ret = await self.run_submission("PrintAndWait")
        self.send_feedback(f"PrintAndWait returned {ret}")
        self.send_feedback("Running PublishThings")
        ret = await self.run_submission("PublishThings", "hello world")
        self.send_feedback(f"PublishThings returned {ret}")
        if not self.parent:  # Only recurse once
            self.send_feedback("Oh boy! Its bout to get recursive in here.")
            await self.run_submission("SuperMission")
            self.send_feedback("Recursion works! Thx kev")
        await self.nh.sleep(0.5)
        return "Woo! Supermissions are awesome!"
