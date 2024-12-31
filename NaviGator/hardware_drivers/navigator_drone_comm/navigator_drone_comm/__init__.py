"""
The :mod:`navigator_drone_comm` module is used to communicate with the drone from Navigator.
Packets received from the drone are published to /navigator_drone_comm/target, /navigator_drone_comm/gps,
and /navigator_drone_comm/target.
Can send packets to the drone by calling the services /navigator_drone_comm/estop, /navigator_drone_comm/stop,
/navigator_drone_comm/start "example mission name".
"""

from .driver import DroneCommDevice
from .packets import (
    Color,
    EStopPacket,
    GPSDronePacket,
    HeartbeatReceivePacket,
    HeartbeatSetPacket,
    Logo,
    StartPacket,
    StopPacket,
    TargetPacket,
    TinPacket,
)
