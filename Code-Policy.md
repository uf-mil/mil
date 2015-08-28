Code Policy
===========

# Adding Code
We follow the [fork-and-pull](https://guides.github.com/activities/contributing-to-open-source/) model of repository code management. 

1. [Fork the repository](https://help.github.com/articles/fork-a-repo/)
2. Make your changes and commit the to that fork
3. Submit a [pull request](https://help.github.com/articles/using-pull-requests/)
4. Assign the pull-request to an appropriate reviewer, or someone random on the team if there is no obvious reviewer. Ask questions at meetings if you have any questions about review policy.
5. Code that is to be run on the sub (Obeying all style-guides, and the MIL internal guides) should go into the master branch. Code that obeys the style-guides, but is still in development, belongs in the dev branch.


There is *exactly one* exception to this policy:
If on pool-test day, there are critical changes that you need to make to the sub, you may commit them directly too the **sub's** onboard git. Only minor changes are permissible at this time. On competition day, all changes must be pair-programmed.

## What should I not commit?
* Do not commit debug print statements (ROS Logging is okay)
* Do not open GUI windows (PCL or OpenCV or anything of that sort) in production code.

## Documentation
* Include a readme in *every* new package, that describes what it is, what it does, and how to use it (in the form of example console commands/switches)
* Add a *wiki* page for every new logical component, that describes in some depth (including citations, dependencies, and alarms) the behavior and uses of that component.
* Someone should be able to use your node or package without needing you to be present
* Python code: Every class and method should have a [docstring](https://en.wikipedia.org/wiki/Docstring#Python). 
    * Class docstring should contain what the class is responsible for/what it encapsulates, caveats, TODO's, and citations
    * Method/function docstrings should contain a list of parameters and their purpose (Someone should be able to determine exactly what would happen if they changed a parameter), and a list of returns and their purpose. It should also include TODO's, and citations, and a summary of methods purpose and behavior.
    * Again, someone should be able to read your docstring and understand the purpose of your code without consulting you
* C++ code: Add docstrings in the same manner as Python, using multi-line comments

## Testing
* Operation critical code shall have unit-tests. Unit tests should fail if you code no longer works
* General code should have unit-tests
* Automated test-by-Simulation (Automatic Hardware-out-of-the-loop testing), is highly recommended for all software
* [C++ unit testing w/ gtest](https://code.google.com/p/googletest/)
* [Python unit testing w/ Nose](https://nose.readthedocs.org/en/latest/)

Unit testing generally saves time for the developer. Despite the initial time spent implementing the test, unit tests make it very easy to make changes to your code, and verify that your code will work.
If you have any questions about what you should unit tests, talk to Patrick Emami.

## Logging
[Take a look at the ROS logging guide](http://wiki.ros.org/roscpp/Overview/Logging)
* Make sure you have good ROS logging, with appropriate log-levels for the information logged
* You should log anything that a human reviewer would want to know in the event of a problem
* Log anything that could be considered fatal, with some description as to what happened
* If your node interacts with hardware, hardware connection/loss, kill/unkill state transitions
* Be diligent in logging, but don't feel that you need to log every single possible state change

## Alarms
* [See the alarm page on this wiki](https://github.com/uf-mil/Sub8/wiki/System-Alarms)
* Every node should raise alarms over the alarm system for indicating problems

# Style Guides

## Python
* [PEP8](https://www.python.org/dev/peps/pep-0008/)
* Enforce PEP8 with [flake8](https://pypi.python.org/pypi/flake8), or use the flake8 linter Subplime plugin.
* We will follow PEP8 with one exception: the line length may be up to 100 characters

## C++ 
* [Google Styleguide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html)
* [Google styleguide linter](https://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py) or find a Sublime plugin/linter that enforces the google styleguide


