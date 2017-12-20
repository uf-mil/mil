#!/usr/bin/env python
import unittest
import sys
import inspect
from mil_tasks_core import BaseTask


class TasksImportTest(unittest.TestCase):
    def __init__(self, *args):
        self.module = sys.argv[1]
        self.base_class = sys.argv[2]
        print(self.base_class, self.module)
        super(TasksImportTest, self).__init__(*args)

    def test_import_tasks(self):
        try:
            task_module = __import__(self.module)
        except Exception as e:
            self.fail(msg='Exception importing module: {}'.format(e))

        if not hasattr(task_module, self.base_class):
            self.fail(msg='{} doesnt have base task {}'.format(self.module, self.base_class))
        base_task = getattr(task_module, self.base_class)

        for name, cls in inspect.getmembers(task_module):
            if inspect.isclass(cls) and issubclass(cls, BaseTask):
                self.assertTrue(
                    issubclass(cls, base_task),
                    msg='{} is not a subclass of {}'.format(name, self.base_class))

if __name__ == '__main__':
    import rostest
    rostest.rosrun('mil_tasks', 'import_test', TasksImportTest)
