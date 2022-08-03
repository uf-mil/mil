# C++ Style Guide
Welcome to the C++ style guide! This style guide briefly explains how we write
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
style guidelines when you push, it may be denied, requiring you to go fix it.

## Philosophy
Before discussing the nuts and bolts of our desired C++ style, we'll offer a brief
on our "code philosophy". These are guidelines that will generally not be checked
by CI or automated systems. Rather, it's generally up to your friendly fellow MILers
to try to encourage the following of these philosophies.

In general, try to make your code **user-friendly**. This is super important, since
all of us at MIL are humans, not computers! While computers run our code, they never
innovtate on it, they run whatever we tell them. It's up to us to work together
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
in that file without opening it, based on it's name.

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
  void set_speed(int speed);  // Woah, a setter...
  int get_speed();  // and a mutator!
private:
  int speed_;
};
```

#### Variables
Variables (including function arguments) should use snake case. For example:
```cpp
int thisIsNotGood;  // No!
int this_is_good;  // Yes!

void exampleFunction(bool argOne, std::string awesomePhrase);  // No!
void exampleFunction(bool arg_one, std::string awesome_phrase);  // No!
```

Please remember to also be descriptive with your variable names. There's no need
to be cryptic or needlessly short with your variable names.
```cpp
int x;  // What is this supposed to represent?
int rotations;  // Oh, the number of rotations!

bool wsy;  // What is this??
bool was_seen_yesterday;  // Ah, that helps much more!
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
When using floating point numbers in C++, either use an integer to initialize the float,
or use a radix point with numbers on both sides. If you want to use exponential notation
with the number, use either an `e` or an `E`.

```cpp
float f = 1.f;  // Nope: Use numbers on both sides of the radix point.
long double ld = -.5L;  // Same as above;
float f = 1.0f;  // Awesome!
float f2 = 1;  // This works great, too!
long double ld = -0.5L;  // Fantastic work!

double huge = 1254e4;  // Nope: Where's the radix point?
double huge_again = 1254.0e4;  // Nice!
```

#### Function Calls
When using function calls, sometimes you may need to call a function with a ton of
arguments, which can cause the call to become excessively long.

If you need to break up the function call, you can align the rest of the arguments
in the function call with the first argument. If you've already indented, then you
can use a four space indent to align the arguments.

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
bool result = weirdMathFunction(quat[1] + std::sqrt(quat[2]) & 8,  // Crazy Scientist A's matrix determinant formula
                                argument_two, argument_three);
```

Sometimes, you may want specific arguments to exist on specific lines for readability.
This is totally okay, too!
```cpp
int magic[][] = createMagicSquare(x1, x2, x3,
                                  y1, y2, y3,
                                  z1, z2, z3);
```

#### Braces
When using braces for conditions or control logic, place the braces on a new line.
In the case that the logic inside a control block is short (one line or less),
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

#### Switches
For `switch` blocks, braces are optional. Attempt to be consistent throughout a particular
package.

In the case of fallthrough (ie, the exclusion of `break` in a case statement),
please add a comment indicating fallthrough. Some tutorials might suggest the use
of the `[[fallthrough]]` attribute, but unfortunately, this is only allowed in C++17
and further.

```cpp
switch (x) {
  case 1:
    ...
    break;
  case 2:
    ...
    break;
}
// or
switch (x) {
  case 1: {
    ...
    break;
  }
  case 2: {
    ...
    break;
  }
  default: {
    ...
    break;
  }
}
```

#### Loops
Loops are similar to conditional statements. Braces can be avoided for very short
loops, or they can be kept in.

For loops with no bodies, either use a pair of empty braces or use the `continue`
statement to indicate that the loop is continually running.
```cpp
for (int i = 0; i < 10; i++)
  sum += i;

while (tired) {
  drinkCoffee();
}

while (spin_forever) continue;
```

#### Pointers and References
When using pointers and references, place the * or & either before the space, or
after the space. Keep this syntax the same throughout one file.

In some cases, such as using pointers as a template parameter, you can remove the space
entirely.

Never declare more than one pointer on a single line, as this expression is often
misread.

```cpp
int* x;
p = &x;

char *c;
const int& p = &x;

CharClass<char*> example;

int * x;  // No!
const int & a;  // Sad face.
```

#### Preprocessor Directives
Preprocessor directives always start at the beginning of the line, full stop.
This may appear to break up indentation - this is fine! The examination of preprocessor
directives is important, and using significant indentation helps to show this.

You can optionally use spaces after the `#` for nested preprocessor directives.

```cpp
if (test_condition) {
  #if LETS_GO_CRAZY
  startCrazyMode();
  #endif
  cleanup();
}
```

#### Class Structure
When structuring your class definition, use one space to indent the `public`, `protected`,
and `private` keywords. Then, use the traditional two-space indent for all other
needed members in the class.

```cpp
class ExampleClass : public BaseClass {
 public:
  ExampleClass();
  ~ExampleClass() {}

 protected:  // Do not leave a blank line after these keywords
  someFunction();

 private:
  int some_var_;
  char special_char_;
};
```

#### Namespaces
Namespaces do not add an extra level of indentation.

```cpp
namespace cooking {
class Pan {
 public:
  void pourCupcake(int row, int col);
  Cupcake retrieveCupcake(int row, int col);

 private:
  Cupcake[][] cupcakes_;
};
}
```

#### Horizontal Whitespace
General rules about horizontal whitespace:

* Comments at the end of a line should be separated from the contents of the line
  by two spaces.
* Semicolons should generally not have spaces before them.
* The colon in a class initializer list should generally have a space around it.
* Keywords in control blocks (`if`, `for`, `while`, etc) should have a space around them.
* Semicolons inside a `for` statement generally have a space after them. (Ie, `for (int i = 0; i < 5; i += 2)`)
* Colons in range-based for loops generally have spaces on both sides.
* Assignment operators (`x = 0`, `x += 5`)generally have spaces on both sides.
* Binary operators (`x + y`, `x || y`) traditionally have spaces, but these can be removed in select scenarios.
* Unary operators (`!x`, `&x`) almost never have spaces between them and their subject.

#### Vertical Whitespace and Blank Lines
Vertical whitespace is helpful, **sometimes**. Vertical whitespace generally includes
blank lines.

Use blank lines to separate thoughts inside of a particular scope (class definition,
function, etc.).

There is no need to start/end a function with a blank line. Adding a blank line
before a comment can help with readability, as can using a blank line without any
associated comment to split two different ideas.

### C++ Features
C++ comes with a lot of cool features, but these features can sometimes cause problems.
We must be careful about which ones we intend to use.

#### Preprocessor Macros
Don't use them. Macros are likely going to cause us more trouble than they're worth
because of their global scope and unfriendly use.

#### Namespaces and `using`
Namespaces are a great way to separate code. The `using` keyword allows one to use
code from a separate namespace somewhere else. For example:

```cpp
using std::list;
```

When using `using` (did you smile at that? I hope so!), try to keep the statements specific.
Don't try to use all of `std`, which could pollute a file's namespace. Instead,
try to use specific things from `std`.

#### Friends
Everyone loves having friends, and C++ does, too. Use the `friend` keyword cautiously
throughout your C++ code. Where it is used, please try to keep the friend to the same
file so that a user does not have to go look in another file.

It may also be desired to conduct unit tests through the `friend` keyword.

#### Exceptions
Use exceptions judiciously. Exceptions are great for representing true failures
in a particular piece of code, and can allow for the abstraction of error handling methods.

However, exceptions can also introduce weird behavior into a piece of code. Exceptions
can cause functions to return before one thinks they will, and can cause issues if another
function catches a particular error before another function.

User input at runtime (this is rare when working with autonomous vehicles, yes) should
never throw an exception. An exception represents a code failure, not the failure
of one user to input a proper expression into a live piece of code.

#### `const`
Use `const` wherever it is appropriate. `const` can help the compiler ensure that
some classes and methods are only mutating a few select values. Remember though
that `const` can quickly become viral; ie, if you pass a `const` variable to a function,
that function will need `const` in its prototype.

#### Using `sizeof`
When using `sizeof`, prefer the use of a variable as the argument, rather than the use
of a class or type.

### ROS Helpers
#### Assertions
When using assertions in a piece of C++ code, please use a function from the family
of `ROS_ASSERT` functions.

#### Printing
When printing to the console, please use the `rosconsole` family of functions.

#### Deprecation
To mark a function as deprecated, add the `ROS_DEPRECATED` attribute to its signature.
To mark a class as deprecated, mark all of its functions as deprecated.

```cpp
ROS_DEPRECATED int cardCount();

class MyClass {
 public:
  ROS_DEPRECATED MyClass();

  ROS_DEPRECATED void specialFunction();
};
```
