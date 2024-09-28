"""
The :mod:`navigator_ball_launcher` module is used to control the ball launcher
on NaviGator. The module implements the electrical protocol to communicate with
a board that controls the flywheel and servo to drop the balls.
"""

from .driver import BallLauncherDevice
from .packets import ReleaseBallPacket, SetSpinPacket
