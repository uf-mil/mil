#!/usr/bin/env python
import unittest
import sys
import inspect
from mil_missions_core import BaseMission


class MissionsImportTest(unittest.TestCase):
    def __init__(self, *args):
        self.module = sys.argv[1]
        self.base_class = sys.argv[2]
        print(self.base_class, self.module)
        super(MissionsImportTest, self).__init__(*args)

    def test_import_missions(self):
        try:
            mission_module = __import__(self.module)
        except Exception as e:
            self.fail(msg='Exception importing module: {}'.format(e))

        if not hasattr(mission_module, self.base_class):
            self.fail(msg='{} doesnt have base mission {}'.format(self.module, self.base_class))
        base_mission = getattr(mission_module, self.base_class)

        for name, cls in inspect.getmembers(mission_module):
            if inspect.isclass(cls) and issubclass(cls, BaseMission):
                self.assertTrue(
                    issubclass(cls, base_mission),
                    msg='{} is not a subclass of {}'.format(name, self.base_class))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mil_missions', 'import_test', MissionsImportTest)
