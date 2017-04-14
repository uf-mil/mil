The "Monte-Carlo Engine" is a catch-all name for the toolbox we're building for quickly running many tests with randomized initial conditions.

For code like the controller, unit-tests just tell us that each function/class does what it claims. Further, simple integration tests just tell us that everything communicates properly. Enter Monte-Carlo testing. By running many tests, with randomized initial conditions, we can generate a very firm idea of the quality of something like a controller, motion-planner, perception algorithm, etc.

# What does it do?

Right now, the most meaningful tool is a simple automatic controller verification tool, which can be run using ROSTEST to verify that a controller converges to some ball on average. You can configure the success criteria to test whatever behavior you are after (The example is exceptionally loose!). There is plenty of functionality worth adding, and more useful statistical analysis:

    - [ ] Did any trajectories diverge?
        - [ ] If so, what percent? What did the initial conditions look like?
    - [ ] On-CI comparison to previous controller quality, to verify that the controller not only **works**, but also **has not gotten worse**.

We are currently somewhat restricted in the sort of analysis we can do because we are not willing to do hundreds of runs in a single test sitting. So any statistical analysis you plan to do will have to work with 30 runs or less. If you can make the sim run faster, then this won't be necessary.

# Examples

Monte-Carlo's:
This is an automatically generated plot of 20 runs with random initial conditions, all with the same randomly offset gains.

![Monte-Carlo Random Xnought](http://i.imgur.com/YecGkfo.png?1)

The Monte-Carlo approach can be used to verify that a controller has behavior that "makes sense" - but that's just one minor application.

We can test much more rich behaviors, like:
* Behavior of the sub when it's dropped into the water from random heights or random orientations
* Drop N thrusters 10 seconds into a trajectory
* Lose DVL track at a random time in the trajectory

# Considerations

* Test at state in N~(sigma, some_state_X), with 1000 tests
* Test in simulation without requiring rendering
    * We should be able these tests on Semaphore
* Should be able to use them in nosetests, and do everything in Python
* Should be able to use this outside of a unit-test, for gain optimization or whatever