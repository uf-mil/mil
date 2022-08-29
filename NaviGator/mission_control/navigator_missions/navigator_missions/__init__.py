import mil_missions_core

from .navigator import Navigator

ChainWithTimeout = mil_missions_core.MakeChainWithTimeout(Navigator)
Wait = mil_missions_core.MakeWait(Navigator)
del mil_missions_core

from . import pose_editor
from .back_and_forth import BackAndForth
from .circle import Circle
from .circle_tower import CircleTower
from .constant_velocity import ConstantVelocity
from .coral_survey import CoralSurvey
from .demonstrate_navigation import DemonstrateNavigation
from .deploy_thrusters import DeployThrusters
from .detect_deliver import DetectDeliver
from .detect_deliver_find import DetectDeliverFind
from .discount_docking import DiscountDocking
from .docking import Docking
from .entrance_gate import EntranceGate
from .example_mission import ExampleMission
from .explore_towers import ExploreTowers
from .fire_launcher import FireLauncher
from .go_to_poi import GoToPOI
from .gps_points import GPSWaypoints
from .grinch_deploy import GrinchDeploy
from .grinch_retract import GrinchRetract
from .killed import Killed
from .move import Move
from .obstacle_avoid import ObstacleAvoid
from .pinger import PingerMission
from .pinger_andy import PingerAndy
from .pinger_exit import PingerExitMission
from .reload_launcher import ReloadLauncher
from .retract_thrusters import RetractThrusters
from .ring_recovery import RingRecovery
from .shoot_balls import ShootBalls
from .station_hold import StationHold
from .stc_jaxon import ScanTheCodeJaxon
from .teleop import Teleop

# Currently breaks mission server, TODO: fix or delete
# from .track_target import TrackTarget
