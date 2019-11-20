#!/usr/bin/env python
from mil_vision_tools import GaussianColorClassifier
from rospkg import RosPack
import os


class VrxColorClassifier(GaussianColorClassifier):
    CLASSES = ['buoy', 'red_totem', 'green_totem', 'blue_totem', 'yellow_totem',
               'black_totem', 'surmark46104', 'surmark950400', 'surmark950410']

    def __init__(self):
        rospack = RosPack()
        path = rospack.get_path('navigator_vision')
        training_file = os.path.join(path, 'config/vrx/training.csv')
        super(VrxColorClassifier, self).__init__(VrxColorClassifier.CLASSES,
                                                    training_file=training_file)


if __name__ == '__main__':
    '''
    Can be run as executable to extract features or check accuracy score
    '''
    import sys
    s = VrxColorClassifier()
    s.main(sys.argv[1:])
