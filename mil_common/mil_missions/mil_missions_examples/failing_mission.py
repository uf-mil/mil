from .base_mission import ExampleBaseMission


class FailingMission(ExampleBaseMission):
    @classmethod
    async def setup(cls):
        print("PrintAndWait init")

    async def run(self, parameters):
        raise ValueError("This is an example error!")
