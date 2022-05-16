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

## Philosophy
Before discussing the nuts and bolts of our desired C++ style, we'll offer a brief
on our "code philosophy". These are guidelines that will generally not be checked
by CI or automated systems. Rather, it's generally up to your friendly fellow MILers
to try to encourage the following of these philosophies.

In general, try to make your code **user-friendly**. This is super important, since
all of us at MIL are humans, not computers! While computers run our code, they never
innovtate on it, they just run whatever we tell them. It's up to us to work together
to innovate new solutions in our programming so that these robots can do new things.

### 1. Be expressive, not restrictive
(Based on [ISO C++ style P1](http://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#Rp-direct))

When writing code, try not to be cryptic or restrictive. Let's say you're writing
code to find objects in an image and you want to write a method that returns
the position of the first image you find:

```cpp
std::vector<int> SpecialObjectFinder::getLocationFirstImage();
```

What does that `std::vector` do? What does it represent? Let's try again.

```cpp
Position SpecialObjectFinder::getLocationFirstImage();
```

Oh - a position! In the first example, the `std::vector` was actually a vector
of two ints representing a x-value and y-value in the camera image of the center of
the object. That's hard to communicate without external documentation. However,
using another class, `Position` communicates the functions' goal much clearer - it's
meant to return a position. This class has x and y attributes.

In this regard, attempt to create and abstract new data structures when you write
code. If you're code is becoming complicated with exceptions and conditional statements
all over the place, consider asking yourself if there's any way to slim down some of
the functionality. Check out the ISO example above for an example involving finding
an element in an iterator.

Ask yourself how you express _your ideas_ write into the code!
