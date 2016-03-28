Mission Scheduler
==============

The mission scheduler is the component of the vehicle that contains domain specific knowledge about the competition.

# Considerations

* Handle simultaneous actions
* Command a path while commanding the vehicle to do other things
* Compute a plan, given the number of points each task rewards, and the time it will take to complete it
* Write mission scripts in Python, with completion conditions, timeouts, etc


We're using Twisted, tutorials to come. (See some in readme for sub8_missions).


* Handle all vision as txros services/actions; ex:
    * 'vision/buoys/green' or something like that
    * All Deferred
    * Aggressively timestamp images with time-read, so we can abuse the TF cache to get the actual world position of the object

