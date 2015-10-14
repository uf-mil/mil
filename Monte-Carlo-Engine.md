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



Any takers?