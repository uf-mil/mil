import asyncio

from .base_mission import ExampleBaseMission


class FailingMission(ExampleBaseMission):
    async def run(self, parameters):
        raise ValueError("This is an example error!")


class CancelledMission(ExampleBaseMission):
    async def run(self, parameters):
        raise asyncio.CancelledError()
