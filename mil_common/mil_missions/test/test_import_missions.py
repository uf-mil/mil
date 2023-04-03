#!/usr/bin/env python3
import inspect
import sys
import unittest

from mil_missions_core import BaseMission


class MissionsImportTest(unittest.TestCase):
    def __init__(self, *args):
        self.module = sys.argv[1]
        self.base_class = sys.argv[2]
        print(self.base_class, self.module)
        super().__init__(*args)

    def test_import_missions(self):
        try:
            mission_module = __import__(self.module)
        except Exception as e:
            self.fail(msg=f"Exception importing module: {e}")

        if not hasattr(mission_module, self.base_class):
            self.fail(
                msg="{} doesn't have base mission {}".format(
                    self.module,
                    self.base_class,
                ),
            )
        base_mission = getattr(mission_module, self.base_class)

        for name, cls in inspect.getmembers(mission_module):
            if inspect.isclass(cls) and issubclass(cls, BaseMission):
                self.assertTrue(
                    issubclass(cls, base_mission),
                    msg=f"{name} is not a subclass of {self.base_class}",
                )


if __name__ == "__main__":
    import rostest

    rostest.rosrun("mil_missions", "import_test", MissionsImportTest)
