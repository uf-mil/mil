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

## Version
All of code should target the C++11 standard. This is typically enforced in each
package's `CMakeLists.txt` file.

## Style Guide
This section explains the actual style practices we encourage when writing C++
code for MIL.

### Naming
_Source: [ROS C++ style 4](http://wiki.ros.org/CppStyleGuide#Naming), 
[Google Style Naming](https://google.github.io/styleguide/cppguide.html#Naming)_

Below explains the process of naming various resources around the repository.

#### Files
When naming your files, use **underscores**, and attempt to be descriptive, but
not lengthy. Consider if someone new to that package would be able to guess what's
in that file without opening it, just based on it's name.

For the extensions, use `.cpp` for a C++ file, and use `.h` for a header file.
There's no reason to use `.cc` or `.cxx` or `.hpp` or any other extension.

#### Classes/Types
For classes and types, use title case. If the phrase contains an acronym, then feel
free to only use upper letters for the acronym:

```cpp
class DrSchwartz;
class GCPUImplementation;
```

#### Functions
Functions should use camel case. This is like title case, but the first letter is not
capitalized.

```cpp
void undoTransformation();
ExampleClass constructFromMessage();
```

One exception to this rule is the naming guidelines for accessors and mutators.
These can be named similarly to variables:
```cpp
class Car {
public:
  void set_speed(int speed); // Woah, a setter...
  int get_speed(); // and a mutator!
private:
  int speed_;
};
```

#### Variables
Variables (including function arguments) should use snake case. For example:
```cpp
int thisIsNotGood; // No!
int this_is_good; // Yes!

void exampleFunction(bool argOne, std::string awesomePhrase); // No!
void exampleFunction(bool arg_one, std::string awesome_phrase); // No!
```

Please remember to also be descriptive with your variable names. There's no need
to be cryptic or needlessly short with your variable names.
```cpp
int x; // What is this supposed to represent?
int rotations; // Oh, the number of rotations!

bool wsy; // What is this??
bool was_seen_yesterday; // Ah, that helps much more!
```

Furthermore:
* Constants should use all capitals. For example, ``SPEED_OF_LIGHT``. This also
  applies to the values of enumerators.
* Member variables (of classes, not structs) should have a trailing underscore. 
  For example, ``internal_attribute_``.
* Global variables should be prepended with ``g_``. For example, ``g_global_attribute``.

#### Namespaces
Namespaces should use underscores. Namespaces should never be named ``std``, and
common namespace names (such as ``util``) should be avoided. Using these namespaces
can create problems when using fully-qualified lookups in C++. ([What is that?](https://abseil.io/tips/130))

```cpp
namespace large_hadron_collider {
  ...
}
```

### Formatting
_Source: [ROS C++ style: 6](http://wiki.ros.org/CppStyleGuide#Formatting), 
[Google Style: Formatting](https://google.github.io/styleguide/cppguide.html#Formatting)_

#### Line Length
Each of your lines should no more than 120 characters long. Generally, 80-100 characters
is still acceptable, but if you can avoid it, please do.

One may wonder, why does this matter? Generally, this helps users who have smaller
screens and users who prefer to work with split screens. Additionally, very long lines
can be confusing to readers of code who may see a very long line as two split lines.

There are some exceptions to this rule:
* A comment which contains a long string, such as a URL.
* A string which can't be wrapped. For example, a string containing a long URL, or
  a string where the newlines are significant, such as a help message.
* An include statement.
* A header guard.
* A `using` declaration.

If a tool (like `clang-format`) complains about the line length anyways, you will
need to disable the tool on the lines where it is complaining.

#### Indentation
Always use spaces, and always use 2 spaces for indentation.

#### Floating Points
When using floating point numbers in C++, either use an integer to intialize the float,
or use a radix point with numbers on both sides. If you want to use exponential notation
with the number, use either an `e` or an `E`.

```cpp
float f = 1.f; // Nope: Use numbers on both sides of the radix point.
long double ld = -.5L; // Same as above;
float f = 1.0f; // Awesome!
float f2 = 1; // This works great, too!
long double ld = -0.5L; // Fantastic work!

double huge = 1254e4; // Nope: Where's the radix point?
double huge_again = 1254.0e4; // Nice!
```

#### Function Calls
When using function calls, sometimes you may need to call a function with a ton of
arguments, which can cause the call to become excessively long.

If you need to break up the function call, you can align the rest of the arguments
in the function call with the first argument. If you've already indented, then you
can just use a four space indent to align the arguments.

```cpp
bool result = funnyFunction(huge_argument_number_one_oh_my_this_is_long,
                            argument_two, argument_three);

if (...) {
  if (...) {
    while (...) {
      bool result = funnyFunction(huge_argument_number_one_oh_my_this_is_long,
          argument_two, argument_three);
    }
  }
}
```

Sometimes, the arguments of function calls might be complex. In this case, feel
free to put one arugmnet on one line by itself.
```cpp
bool result = weirdMathFunction(quat[1] + std::sqrt(quat[2]) & 8, // Crazy Scientist A's matrix determinant formula
                                argument_two, argument_three);
```

Sometimes, you may just want specific arguments to exist on specific lines for readability.
This is totally okay, too!
```cpp
int magic[][] = createMagicSquare(x1, x2, x3,
                                  y1, y2, y3,
                                  z1, z2, z3);
```

#### Braces
When using braces for conditions or control logic, place the braces on a new line.
In the case that the logic inside a control block is simple (one line or less),
then the braces can be excluded. An exception to this rule is `switch` statements, discussed
below.

```cpp
if (example_term)
{
  ...
}

if (simple_test)
  simpleCall();

if (this_works_too)
{
  simpleCall();
}

while (condition_is_true) continue;
```

For cases where braces are excluded, there should be no other conditional statement
other than the primary statement. For example, a conditional block using no braces
should only use an `if` statement and should not have an `else if` or `else` clause.

#### Conditionals
Conditional blocks should always use spaces to break up key terms. For example,
the `if` keyword and the expression inside of the parentheses following the `if`
keyword should be separated by a space. This likewise applies to `else if` and `else`.

```cpp
if (this_is_perfect) {
  // Great job!
  ...
} else if (wow_you_did_it == again) {
  ...
}
```
