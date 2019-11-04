#!/usr/bin/env python
from mil_vision_tools import GaussianColorClassifier
from rospkg import RosPack
import os


class ScanTheCodeClassifier(GaussianColorClassifier):
    CLASSES = ['stc_panel_off', 'stc_panel_red', 'stc_panel_green', 'stc_panel_blue', 'stc_panel_yellow']

    def __init__(self):
        rospack = RosPack()
        path = rospack.get_path('navigator_vision')
        training_file = os.path.join(path, 'config/stc/training.csv')
        labelfile = os.path.join(path, 'config/stc/labels.json')
        super(ScanTheCodeClassifier, self).__init__(ScanTheCodeClassifier.CLASSES,
                                                    training_file=training_file, labelfile=labelfile)


if __name__ == '__main__':
    '''
    Can be run as executable to extract features or check accuracy score
    '''
    import sys
    s = ScanTheCodeClassifier()
    s.main(sys.argv[1:])
