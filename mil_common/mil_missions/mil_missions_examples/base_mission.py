#!/usr/bin/env python
from mil_missions_core import BaseMission


class ExampleBaseMission(BaseMission):
    @classmethod
    def _init(cls, mission_runner):
        super(ExampleBaseMission, cls)._init(mission_runner)
        # Establish robot specific subscribers, tools, etc here
        print 'ExampleBaseMission initialized!'
