#!/usr/bin/env python3
import os

from mil_vision_tools import GaussianColorClassifier
from rospkg import RosPack


class VrxStcColorClassifier(GaussianColorClassifier):
    CLASSES = ["off", "red", "green", "blue"]

    def __init__(self):
        rospack = RosPack()
        path = rospack.get_path("navigator_vision")
        training_file = os.path.join(path, "config/vrx/stc_training.csv")
        super(VrxStcColorClassifier, self).__init__(
            VrxStcColorClassifier.CLASSES, training_file=training_file
        )


if __name__ == "__main__":
    """
    Can be run as executable to extract features or check accuracy score
    """
    import sys

    s = VrxStcColorClassifier()
    s.main(sys.argv[1:])
