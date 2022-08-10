#!/usr/bin/env python3
from mil_missions_core import BaseMission


class ExampleBaseMission(BaseMission):
    @classmethod
    def _init(cls, mission_runner):
        super()._init(mission_runner)
        # Establish robot specific subscribers, tools, etc here
        print("ExampleBaseMission initialized!")
