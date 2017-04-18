This policy was conceived of and written by Jacob Panikulam and Patrick Emami. You know where to find us if you want to start a fight.

## Adding Code
We follow the [fork-and-pull](https://guides.github.com/activities/contributing-to-open-source) model of repository code management.

Before submitting changes, read the rest of this page. Any code that is pull-requested should *absolutely never* break catkin_make for master. Test locally before pull-requesting. Incomplete code should be pull-requested to a feature branch (It's okay if this doesn't pass unit-tests, but it does have to make).

1. [Fork the repository](https://help.github.com/articles/fork-a-repo)
2. Make your changes and commit the to that fork
    * Verify that your changes compile and pass a flake8 test
3. Submit a [pull request](https://help.github.com/articles/using-pull-requests)
4. Assign the pull-request to an appropriate reviewer, or someone random on the team if there is no obvious reviewer. Ask questions at meetings if you have any questions about review policy.
    * You may not see movement on your PR for some time, as the reviewer waits for it to build on our continuous integration server
    * MAKE SURE you are a collaborator on SemaphoreCI. If the CI build fails, everyone will get a notification, and will likely not respond to your PR on the assumption that you noticed the failed build.
5. Code that is to be run on a vehicle (Obeying all style-guides, and the MIL internal guides) should go into the master branch. Code that obeys the style-guides, but is still in development, belongs into your own feature branch (ex: sim, monte-carlo, controller), to be merged when complete.


## Review
* Very minor changes (single line or less) require no serious review if you have push access
* Somewhat minor changes should remain in review for no more then 24 hours (After which, pull so long as build is passing)
    * This is to prevent blocking progress on innocuous changes
* When your code is reviewed and approved, it will be pulled into the upstream repository
* If your code is not approved, the following must happen
    * If your pull-request is multiple commits, add a new commit that makes the suggested changes
    * If your pull-request is a single commit, [amend](https://www.atlassian.com/git/tutorials/rewriting-history/git-commit--amend) that commit with the suggested changes
* If you are reviewing code, use github's line-comment feature on their commits to outline issues with an individual line
* Broader issues should be outlined in a comment on the pull-request
* Reviewers: Be sure to manually read the SemaphoreCI output, there may have been a quiet failure that didn't kill the build

## Commit Messages
* Follow [this guide](http://tbaggery.com/2008/04/19/a-note-about-git-commit-messages.html) for writing a good commit message
    * The first part of the message should be under 50 characters (So that it fits in the summary on github), then the remainder can be any length.
    * Commits should be capitalized and properly spelled
    * The commit message should outline all of the major changes made, and reference the issue if applicable
* Every commit message should be prepended with a topic, ex:

`git commit -m CONTROLLER: Implement unit tests"`

`git commit -m "AZI_DRIVE: Add CVXGEN solver"`

* Every commit should represent a logical unit, both in code and in message, never just "work" or "fix"
* [Don't do this](http://www.commitlogsfromlastnight.com)
* Include a brief line about how you tested your code

## Merging and Rebasing
* If you have made commits **locally**, and want to take in commits from upstream, you should `rebase`

    git pull --rebase upstream master

* If you have made commits on your feature branch, and some of these commits exist on the uf-mil remote, you should `merge

    git pull --no-ff upstream master  # This will merge with no fast-forwarding

* You *DO NOT* need to merge master into your branch before submitting a pull request, the PR will do the merging.


## What should I not commit?
* Do not commit debug print statements (ROS Logging is okay)
* Do not open GUI windows (PCL or OpenCV or anything of that sort) in production code. This will often cause crashes
* Avoiding unneeded merging when pulling code, try to  rebase `git pull --rebase upstream master` unless impossible
* If you must merge, you should always merge --no-ff to preserve history

## Documentation
* Your code must be attributed, if you borrow it from other projects, or use code from stackoverflow, etc
* Include a readme in *every* new package, that describes what it is, what it does, and how to use it (in the form of example console commands/switches)
* Add a *wiki* page for every new logical component, that describes in some depth (including citations, dependencies, and alarms) the behavior and uses of that component (i.e. how it works, what knowledge is necessary to make improvements).
    * Feel free to edit the wiki at any time. Adding a page will require you to edit the sidebar manually.
* In short: Readme's are for how-to-use, Wiki pages should be how-does-this-work
* Someone should be able to use your node or package without needing you to be present
* Python code: Every nontrivial class and method should have a [docstring](https://en.wikipedia.org/wiki/Docstring#Python).
    * Class docstring should contain what the class is responsible for/what it encapsulates, caveats, TODO's, and citations
    * Method/function docstrings should contain a list of parameters and their purpose (Someone should be able to determine exactly what would happen if they changed a parameter), and a list of returns and their purpose. It should also include TODO's, and citations, and a summary of methods purpose and behavior.
    * Again, someone should be able to read your docstring and understand the purpose of your code without consulting you
* C++ code: Add docstrings in the same manner as Python, using multi-line comments

## Adding Dependencies
* If you add a new library that your code depends on, that is not installed with ros-kinetic-desktop-full or Ubuntu 16.04 by default, you MUST note it in your readme and somewhere on the wiki if applicable.
    * New dependencies also need to be added to the automated install process. Add a line for them under in the correct block of commands to the install.sh script in the [installer repository](https://github.com/uf-mil/installer). If you are unsure about how to do this, open an issue and request for a senior team member to add it.
* If you submit a PR that requires some new library without updating the install script, the Semaphore build will fail and yell at you.

## Testing
* We use SemaphoreCI to automatically test pull-requests and new commits to our repositories. You still need to test your code locally, but know that everyone can view make and unit-test results on Semaphore before considering your pull-request.
* Operation critical code shall have unit-tests. Unit tests should fail if your code no longer works
* General code should have unit-tests
* Automated test-by-Simulation (Automatic Hardware-out-of-the-loop testing), e.g. Monte-Carlo testing is highly recommended for all software
* [C++ unit testing with gtest](https://code.google.com/p/googletest)
* [Python unit testing with Nose](https://nose.readthedocs.org/en/latest)

Unit testing generally saves time for the developer. Despite the initial time spent implementing the test, unit tests make it very easy to make changes to your code, and verify that your code will work.
If you have any questions about what you should unit tests, talk to Patrick Emami.

## Alarms
* [See the ros_alarms wiki](https://github.com/uf-mil/ros_alarms/wiki)
* Every node should raise alarms over the alarm system for indicating problems

## Python Style Guide
* [PEP8](https://www.python.org/dev/peps/pep-0008)
* Enforce PEP8 with [flake8](https://pypi.python.org/pypi/flake8), or use the flake8 linter Subplime plugin.
* Use (We ignore E201)

    `flake8 --max-line-length=100 ./`

* We will follow PEP8 with one exception: the line length may be up to 100 characters
* Your code will not be pulled if it is not PEP8 compliant

## C++ Style Guide
* [Clang-format](https://github.com/rosshemsley/SublimeClangFormat) to automatically format your code
    * We use the Google styleguide, because LLVM style breaks nested template declarations
    * We set the maximum line length to 130
* [Google Styleguide](https://google.github.io/styleguide/cppguide.html)
* [Google styleguide linter](https://raw.githubusercontent.com/google/styleguide/gh-pages/cpplint/cpplint.py) or find a Sublime plugin/linter that enforces the google styleguide