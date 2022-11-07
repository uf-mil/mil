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

## Good testing practices

While testing at all is awesome, having especially extensive unit tests can be
very helpful in catching errors quickly. Unit tests can be improved by incorporating
more of these principles.

### Fuzzy and mutation testing

One practice for writing strong unit tests includes the use of mutation and/or
fuzzy testing. In this testing practice, you purposefully "break" the program with
the hope that the tests catch the error successfully. You can do this manually,
or with the help of tools.

To manually fizz code, begin by thinking what could easily break your code. For example,
if you use a string as an argument to a function or class' constructor, what if you
supply a string that's 20,000 lines long? Does the class completely crash unexpectedly,
or does it fail in an expected way? What if you use an empty string, or if you
use a number instead of a string?

Furthermore, you can use tools to help you catch errors that should be added. `mutmut`
is a Python tool that implements a mutation testing framework. You can run the tool
over a specific package and its tests to identify areas that are not covered by tests.

```bash
$ mutmut run --runner "rostest package_name package_test.test" \
             --paths-to-mutate src \
             --tests-dir test # Run mutation tester
$ mutmut html # Create an HTML file showing what happened during testing
$ xdg-open html/index.html # Browse the HTML file to determine what needs to be improved
```

## Python unit tests with {mod}`unittest`

To write unit tests in python, you should use {mod}`unittest`. The structure of a unit
test looks like this:

```python
import unittest
import rostest
from calculator_package import Calculator

class NumTest(unittest.TestCase):
    def setUp(self):
        self.calculator = Calculator()

    def test_add(self):
        self.assertEqual(self.calculator.add(1, 2), 3)
        self.assertEqual(self.calculator.add(-1, 0), -1)

    def test_sub(self):
        self.assertEqual(self.calculator.subtract(1, 2), -1)
        self.assertEqual(self.calculator.subtract(10, 7), 3)

    def tearDown(self):
        self.calculator.cleanup()

if __name__ == "__main__":
    rostest.rosrun("calculator_package", "test_basic_operations", NumTest)
    unittest.main()
```

In the example, there are several important pieces:
1. {meth}`~.unittest.TestCase.setUp()` and {meth}`~.unittest.TestCase.tearDown()`
   are called before and after each test, respectively. These methods can be used
   to set up and clean up resources, as needed.
1. Each test is a method that begins with `test_`. Methods that do not begin with
   `test_` will not register as tests.
1. You can use `assertX` methods to assert that the results of operations are
   expected. Example method names include {meth}`~.unittest.TestCase.assertEqual`,
   {meth}`~.unittest.TestCase.assertTrue`, and {meth}`~.unittest.TestCase.assertIn`.

You can also write unit tests for asynchronous operations. Check out our example
from our {mod}`txros` module:

```python
import txros
import unittest
import rostest

class BasicNodeHandleTest(unittest.IsolatedAsyncioTestCase):
    """
    Tests basic nodehandle functionality.
    """

    nh: txros.NodeHandle

    async def asyncSetUp(self):
        self.nh = txros.NodeHandle.from_argv("basic", always_default_name = True)
        await self.nh.setup()

    async def test_name(self):
        self.assertEqual(self.nh.get_name(), "/basic")

    async def asyncTearDown(self):
        await self.nh.shutdown()

if __name__ == "__main__":
    rostest.rosrun("txros", "test_basic_nodehandle", BasicNodeHandleTest)
    unittest.main()
```

## Integrating tests with ROS

After writing tests with `unittest` or `gtest`, you need to actually integrate
them with ROS. This allows them to be tested in our continuous integration systems.

To begin, make a `test/` directory in the package where the test lies. In this test
folder, write a `.test` file in XML. This test file is actually a launch
file, so you have a lot of flexibility in what launches when the test is executed.
However, the most important tag is the `<test>` tag. This specifies what tests
are actually ran and analyzed for their output.

Below is an example test file named `txros.test`. This test file launches one test
file named `test_basic_nodehandle.py`. At its core, this is all you need to run
tests from a test file. You can optionally also launch nodes, set parameters, and
use conditionals, as you can with any launch file.

```xml
<launch>
    <test test-name="test_basic_nodehandle" pkg="txros" type="test_basic_nodehandle.py" time-limit="180.0"/>
</launch>
```

You can see that the `<test>` tag allows us to specify the name of the test, the package
where the test can be found, and most importantly its `type`, or the name of the
executable that runs the test. We can also add a time limit to the test to ensure
that it doesn't run forever.

The next step is to add recognition of this test file into your package's `CMakeLists.txt`
file. You can do so by adding a few lines to this file:

```cmake
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(test/txros.test)
endif()
```

The argument to the `add_rostest` function is the name of your package's test file.

Now, you should be able to see your tests when you compile:
```
$ catkin_make # Run catkin_make to compile everything
$ catkin_make run_tests # Now, run all tests! In the output, you should see your test being ran.
```
