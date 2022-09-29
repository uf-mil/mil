# Writing Unit Tests with `rostest`

An important part of developing software for autonomous systems is testing. Testing
is integral to ensuring that software works in a variety of environments under
a variety of conditions; without tests, it's common to write mistakes in software
that can lead to the failure of systems.

ROS, the middleware powering our robots, provides great interfaces to writing
unit tests for C++ and Python programs, using `gtest` and `unittest` as the testing
libraries, respectively.

## Why write tests?

Why is it even important to write tests for our software? Why is it important to
write tests for software that already _seems_ to work?

Here's a great answer, from the [ROS wiki page on Unit Testing](http://wiki.ros.org/Quality/Tutorials/UnitTesting):
> 1. **You can make incremental updates to your code more quickly.** We have hundreds of packages with many interdependencies, so it's hard to anticipate the problems a small change might cause. If your change passes the unit tests, you can be confident that you haven't introduced problems — or at least the problems aren't your fault.
> 1. **You can refactor your code with greater confidence.** Passing the unit tests verifies that you haven't introduced any bugs while refactoring. This gives you this wonderful freedom from change fear! You can actually make things good quality!
> 1. **It leads to better designed code.** Unit tests force you to write your code so that it can be more easily tested. This often means keeping your underlying functions and framework separate, which is one of our design goals with ROS code.
> 1. **They prevent recurring bugs (bug regressions).** It's a good practice to write a unit test for every bug you fix. In fact, write the unit test before you fix the bug. This will help you to precisely, or even deterministically, reproduce the bug, and much precisely understand what the problem is. As a result, you will also create a better patch, which you can then test with your regression test to verify that the bug is fixed. That way the bug won't accidentally get reintroduced if the code gets modified later on. Also, whoever should accept your patch in a pull request, they will be much more easy to convince that the problem is solved, and the contribution is of high quality.
> 1. **They let you blame other people (contract-based development).** A unit test is documentation that your code meets its expected contract. If your contribution passes the tests written by others, you can claim that you did your job right. If someone else's code fails tests, you can reject it as being not of sufficient quality.
> 1. **Other people can work on your code more easily (an automatic form of documentation).** It's hard to figure out whether or not you've broken someone else's code when you make a change. The unit tests are a tool for other developers to validate their changes. Automatic tests document your coding decisions, and communicate to other developers automatically about their violation. Thus tests become documentation for your code — a documentation that does not need to be read for the most time, and when it does need to be inspected the test system will precisely indicate what to read (which tests fail). By writing automatic tests you make other contributors faster. This improves the entire ROS project.
> 1. **It is much easier to become a contributor to ROS if we have automated unit tests.** It is very difficult for new external developers to contribute to your components. When they make changes to code, they are often doing it in the blind, driven by a lot of guesswork. By providing a harness of automated tests, you help them in the task. They get immediate feedback for their changes. It becomes easier to contribute to a project, and new contributors to join more easily. Also their first contributions are of higher quality, which decreases the workload on maintainers. A win-win!
> 1. **Automatic tests simplify maintainer-ship.** Especially for mature packages, which change more slowly, and mostly need to be updated to new dependencies, an automatic test suite helps to very quickly establish whether the package still works. This makes it much easier to decide whether the package is still supported or not.
> 1. **Automatic tests amplifying Value of Continuous Integration.** Regression tests, along with normal scenario-based requirements tests contribute to overall body of automated tests for your component. This increases effectiveness of the build system and of continuous integration (CI). Your component is better tested against evolution of other APIs that it depends on (CI servers will tell you better and more precisely what problems develop in your code).
