#!/usr/bin/env python
import unittest


class ImportMissionsTest(unittest.TestCase):
    '''
    Simple test to pickup on import / syntax errors in Navigator singleton or individual missions.

    TODO: implement test for basic mission in SIM / with bag data (ex: move forward 1m)
    '''
    def test_import_navigator(self):
        err = None
        try:
            from navigator_singleton.navigator import Navigator  # noqa: F401
        except Exception as e:
            err = e
        self.assertIsNone(err, msg="Error importing Navigator: {}".format(err))

    def test_import_missions(self):
        import os
        import rospkg
        missions = {}
        for module in os.listdir(os.path.join(rospkg.RosPack().get_path("navigator_missions"), 'nav_missions/')):
            if module[0] == '_' or module[-3:] != '.py':
                continue
            missions[module[:-3]] = None
            try:
                __import__('nav_missions', locals(), globals(), [module[:-3]])
            except Exception as e:
                missions[module[:-3]] = e
        for mission in missions:
            self.assertIsNone(missions[mission], msg="Error with mission {}: {}".format(mission, missions[mission]))


if __name__ == '__main__':
    import rostest
    rostest.rosrun('navigator_missions', 'import_missions', ImportMissionsTest)
