import mil_missions_core

from .base_mission import ExampleBaseMission
from .failing_mission import FailingMission
from .print_and_wait import PrintAndWait
from .publish_things import PublishThings
from .super_mission import SuperMission

ChainWithTimeout = mil_missions_core.MakeChainWithTimeout(ExampleBaseMission)
Wait = mil_missions_core.MakeWait(ExampleBaseMission)
