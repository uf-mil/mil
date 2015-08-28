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

## Logging
[Take a look at the ROS logging guide](http://wiki.ros.org/roscpp/Overview/Logging)
* Make sure you have good ROS logging, with appropriate log-levels for the information logged
* You should log anything that a human reviewer would want to know in the event of a problem
* Log anything that could be considered fatal, with some description as to what happened
* If your node interacts with hardware, hardware connection/loss, kill/unkill state transitions
* Be diligent in logging, but don't feel that you need to log every single possible state change

# Style Guides

## Python
* [PEP8](https://www.python.org/dev/peps/pep-0008/)
* Enforce PEP8 with [flake8](https://pypi.python.org/pypi/flake8), or use the flake8 linter Subplime plugin.
* We will follow PEP8 with one exception: the line length may be up to 100 characters

## C++ 
* [Google Styleguide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html)
* [Google styleguide linter](https://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py) or find a Sublime plugin/linter that enforces the google styleguide


