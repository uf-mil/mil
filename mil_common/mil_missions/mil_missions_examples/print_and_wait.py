from .base_mission import ExampleBaseMission


class PrintAndWait(ExampleBaseMission):
    @classmethod
    def init(cls):
        print("PrintAndWait init")

    async def run(self, parameters):
        self.send_feedback("I like eggs")
        await self.nh.sleep(1.0)
        self.send_feedback("I like milk")
        await self.nh.sleep(1.0)
        self.send_feedback("I fear death.")
        return "The darkness isn't so scary"
