"""Handles the actions of the ScanTheCode Mission."""
import numpy as np
import datetime
___author___ = "Tess Bianchi"


class ScanTheCodeAction(object):
    """A helper class for Scan The Code Mission."""

    def __init__(self):
        """Initialize Scan The Code Mission."""
        self.distance = 7

    def initial_position(self, scan_the_code, distance=8):
        """
        Get the initial position that the boat should go.

        Returns the position that you want the boat and the position of scan the code.
        """
        self.distance = distance
        position = [scan_the_code.position.x, scan_the_code.position.y, scan_the_code.position.z]
        now = datetime.datetime.now()
        now_time = now.time()
        if now_time < datetime.time(12, 00):
            return [position[0] - self.distance, position[1], position[2]], position
        else:
            return [position[0] + self.distance, position[1], position[2]], position

    def correct_pose(self, scan_the_code, pose):
        """
        Correct the pose of the boat if it is looking at a corner of scan the code.

        Returns the new correct position of the boat
        """
        position = pose
        stc_position = np.array([scan_the_code.position.x, scan_the_code.position.y, scan_the_code.position.z])
        d = np.linalg.norm(position - stc_position)
        point_a = position[0] + d * np.sqrt(3)
        dir_vec = (stc_position - point_a) / np.linalg.norm(stc_position - point_a)
        return position + dir_vec * self.distance
