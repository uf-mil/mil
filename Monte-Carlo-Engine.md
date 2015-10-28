Monte Carlo Engine
=================

The "Monte-Carlo Engine" is a catch-all name for the toolbox we're building for quickly running many tests with randomized initial conditions.

For code like the controller, unit-tests just tell us that each function/class does what it claims. Further, simple integration tests just tell us that everything communicates properly. Enter Monte-Carlo testing. By running many tests, with randomized initial conditions, we can generate a very firm idea of the quality of something like a controller, motion-planner, perception algorithm, etc.


# Considerations

* Test at state in N~(sigma, some_state_X), with 1000 tests
* Test in simulation without requiring rendering
    * We should be able these tests on Semaphore
* Should be able to use them in nosetests, and do everything in Python
* Should be able to use this outside of a unit-test, for gain optimization or whatever


# Examples

Monte-Carlo's:
This is an automatically generated plot of 20 runs with random initial conditions, all with the same randomly offset gains.

![Monte-Carlo Random Xnought](http://i.imgur.com/YecGkfo.png?1)

The Monte-Carlo approach can be used to verify that a controller has behavior that "makes sense" - but that's just one minor application.

We can test much more rich behaviors, like:
* Behavior of the sub when it's dropped into the water from random heights or random orientations
* Drop N thrusters 10 seconds into a trajectory
* Lose DVL track at a random time in the trajectory