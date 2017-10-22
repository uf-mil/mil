#!/usr/bin/env python
from mil_tasks_core import BaseTask


class ExampleBaseTask(BaseTask):
    @classmethod
    def _init(cls, task_runner):
        super(ExampleBaseTask, cls)._init(task_runner)
        # Establish robot specific subscribers, tools, etc here
        print 'ExampleBaseTask initialized!'
