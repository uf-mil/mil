Code Policy
===========

# Adding Code
We follow the [fork-and-pull](https://guides.github.com/activities/contributing-to-open-source/) model of repository code management. 

Before submitting changes, read the rest of this page. Any code that is pull-requested should *absolutely never* break catkin_make for master. Test locally before pull-requesting. Incomplete code should be pull-requested to the dev branch, and it should not break the dev branch.

1. [Fork the repository](https://help.github.com/articles/fork-a-repo/)
2. Make your changes and commit the to that fork
3. Submit a [pull request](https://help.github.com/articles/using-pull-requests/)
4. Assign the pull-request to an appropriate reviewer, or someone random on the team if there is no obvious reviewer. Ask questions at meetings if you have any questions about review policy.
    * You may not see movement on your PR for some time, as the reviewer waits for it to build on Semaphore, the continuous integration service that we use
    * MAKE SURE you are a collaborator on SemaphoreCI. If the CI build fails, everyone will get a notification, and will likely not respond to your PR on the assumption that you noticed the failed build.
5. Code that is to be run on the sub (Obeying all style-guides, and the MIL internal guides) should go into the master branch. Code that obeys the style-guides, but is still in development, belongs in the dev branch.

There is *exactly one* exception to this policy:
If on pool-test day, there are critical changes that you need to make to the sub, you may commit them directly too the **sub's** onboard git. Only minor changes are permissible at this time, this still does not belong in the main sub8/master. On competition days, *all* changes must be pair-programmed.

## Review
* When your code is reviewed and approved, it will be pulled into the main sub8 repository
* If your code is not approved, the following must happen
    * If your pull-request is multiple commits, add a new commit that makes the suggested changes
    * If your pull-request is a single commit, [amend](https://www.atlassian.com/git/tutorials/rewriting-history/git-commit--amend) that commit with the suggested changes
* If you are reviewing code, use github's line-comment feature on their commits to outline issues with an individual line
* Broader issues should be outlined in a comment on the pull-request

## Commit Messages
* Follow [this guide](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html) for writing a good commit message
    * The first part of the message should be under 50 characters (So that it fits in the summary on github), then the remainder can be any length.
    * Commits should be capitalized and properly spelled
    * The commit message should outline all of the major changes made, and reference the issue if applicable
* Every commit message should be prepended with a topic, ex:

`git commit -m CONTROLLER: Implement unit tests"`

`git commit -m "AZI_DRIVE: Add CVXGEN solver"`

* Every commit should represent a logical unit, both in code and in message, never just "[work](https://github.com/uf-mil/SubjuGator/commits/master?page=4)" or "fix"
* [Don't do this](http://www.commitlogsfromlastnight.com/)
* Include a brief line about how you tested your code

## What should I not commit?
* Do not commit debug print statements (ROS Logging is okay)
* Do not open GUI windows (PCL or OpenCV or anything of that sort) in production code. This will often cause crashes
* If you have many small commits, [squash](http://stackoverflow.com/questions/5189560/squash-my-last-x-commits-together-using-git) your commits before pull-requesting (Making many small commits is good, but squash them before pull-requesting)
* Do not merge when pulling code, always rebase `git pull --rebase` unless impossible

## Documentation
* Include a readme in *every* new package, that describes what it is, what it does, and how to use it (in the form of example console commands/switches)
* Add a *wiki* page for every new logical component, that describes in some depth (including citations, dependencies, and alarms) the behavior and uses of that component (i.e. how it works, what knowledge is necessary to make improvements).
    * Feel free to edit the wiki at any time. Adding a page will require you to edit the sidebar manually.
* In short: Readme's are for how-to-use, Wiki pages should be how-does-this-work
* Someone should be able to use your node or package without needing you to be present
* Python code: Every class and method should have a [docstring](https://en.wikipedia.org/wiki/Docstring#Python). 
    * Class docstring should contain what the class is responsible for/what it encapsulates, caveats, TODO's, and citations
    * Method/function docstrings should contain a list of parameters and their purpose (Someone should be able to determine exactly what would happen if they changed a parameter), and a list of returns and their purpose. It should also include TODO's, and citations, and a summary of methods purpose and behavior.
    * Again, someone should be able to read your docstring and understand the purpose of your code without consulting you
* C++ code: Add docstrings in the same manner as Python, using multi-line comments

## Adding Dependencies
* If you add a new library that your code depends on, that is not installed with ROS-desktop-full or Ubuntu by default, you MUST note it in the wiki, your readme, and finally, add it to the SemaphoreCI build script. 
    * Once you are a collaborator on Semaphore (Ask someone to add you), there are some easy tutorials there for editing the setup script. Unlike travis-ci, the build script only exists on SemaphoreCI, not in our repository.
* If you submit a PR that requires some new library without updating the SemaphoreCI setup-build script, the Semaphore build will fail and yell at you.

## Testing
* We use SemaphoreCI to automatically test pull-requests and new commits to the uf-mil/Sub8 repo. You still need to test your code locally, but know that everyone can view make and unit-test results on Semaphore before considering your pull-request.
* Operation critical code shall have unit-tests. Unit tests should fail if you code no longer works
* General code should have unit-tests
* Automated test-by-Simulation (Automatic Hardware-out-of-the-loop testing), e.g. Monte-Carlo testing is highly recommended for all software
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
* [Clang-format](https://github.com/rosshemsley/SublimeClangFormat) to automatically format your code
* [Google Styleguide](https://google-styleguide.googlecode.com/svn/trunk/cppguide.html)
* [Google styleguide linter](https://google-styleguide.googlecode.com/svn/trunk/cpplint/cpplint.py) or find a Sublime plugin/linter that enforces the google styleguide


# Questions?

If you have any questions, talk to Jake, don't just go committing willy-nilly to master!

This policy was conceived of and written by Jacob Panikulam and Patrick Emami. You know where to find us if you want to start a fight.