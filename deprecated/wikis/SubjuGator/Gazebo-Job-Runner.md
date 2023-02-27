# Running a job

```shell
rosrun subjugator_gazebo job_runner.py my_job_name --iterations 10
```

Where my_job_name is the name of the python file containing your job, and iterations is the number of iterations you'd like to run.

# Writing a job

All jobs belong in SubjuGator/simulation/subjugator_gazebo/diagnostics/gazebo_tests. Name the file what you want the job to be called.

You must create a class called "Job" that inherits from diagnostics.gazebo_tests.common.Job. This provides some nice tools, like setting model position without having to fuss with kills, getting true world position, among other things.

At a minimum, it looks like this

```python
import txros
from twisted.internet import defer
import numpy as np
from diagnostics.gazebo_tests import common

class Job(common.Job):
    _job_name = 'my_special_job'

    @txros.util.cancellableInlineCallbacks
    def setup(self):
        print 'setting up'

    @txros.util.cancellableInlineCallbacks
    def run(self, sub):
        yield defer.returnValue((success, reason))


```