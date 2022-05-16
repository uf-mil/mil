# C++ Style Guide
Welcome to the C++ style guide! This stlye guide briefly explains how we write
our code to do crazy robot things, and how we'd like you to write yours, too!
This style is used in our continuous integration process to make sure that all
code submitted to our repository is somewhat organized and maintainable.

Don't feel pressured to read this entire guide line-by-line and write down every
little note. We have tools that help you follow this style without you knowing most
of the style. However, this guide is meant to serve as the de-facto standard for
our C++ style, so if you have a question as to why something is formatted, you
can propose a change to this guide and the tools that help to enforce it.

## Where it Started
Most of this style guide comes from a combination of the [ROS style guide](http://wiki.ros.org/CppStyleGuide)
and the [Google C++ style guide](https://google.github.io/styleguide/cppguide.html).
These guides are extensive, and generally complement each other, as the ROS style
guide is loosely based on the Google style guidelines. Additionally, some of the more
philosophical aspects may be pulled from the [ISO C++ style guide](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines).

## Enforcement
While being able to have a style reference is important, how are these guidelines
actually enforced? Generally, enforcement is done in two areas:

* [Pre-commit](https://pre-commit.com) (before you push)
* Continuous integration (when you open a pull request)

Pre-commit is a new tool to MIL that will be pushed with the Noetic + Alabaster project.
Pre-commit is a tool that executes a script right before you commit. This tool
can run ``clang-format``, a tool dedicated to formatting and enforcing style on
C++ style. Pre-commit will be expanded on more when the Noetic + Alabaster project
is officially done.

Continuous integration is the service that makes sure that new code added to the repository
is ready to be added. Continuous integration runs a plethora of checks, including
style checks on C++ code. This means that if your code does not comply with the C++
style guidelines when you push, it may be deined, requiring you to go fix it.
