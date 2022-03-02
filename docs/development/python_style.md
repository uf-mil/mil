## Python Style Guide
Welcome to the Python style guide for MIL. This guide is meant to be a relaxed, easy-to-follow guide with some tips on how to make your Python look snazzy! It's based on the [ROS style guide](http://wiki.ros.org/PyStyleGuide) and the [Google Python style guide](https://google.github.io/styleguide/pyguide.html).

Don't feel pressured to know or understand every rule, and totally feel free to skim over the guide at first. This guide can provide some helpful information before you begin to write your Python, but it also serves as an excellent place to determine how to use a particular Python feature in your code.

Additionally, this guide is not meant to be permanent! If you have a suggestion, feel free to bring it up and change the guide! Have fun!

### The Power to Change Things

Before we dive into the Python, a brief word about our code and your ability to change things.

These guidelines are not a hard rule. These guidelines are not permanent, nor should they be. These guidelines were not created by anyone smarter than you.

Therefore, you always have the power to change them. Likewise, if you see code that breaks these guidelines, feel free to change it. If you have suggestions for the guidelines, you should suggest them.

Your innovation and willingness to break and bend the rules that currently exists is what can keep our code powerful, clean, and beautiful.

### Features
This section explains how to use several of Python's features to your advantage.

#### Naming
**Naming** describes how variables, methods, classes, etc. are named. 

##### General
Here is a brief table explaining the general pattern of naming:

<table rules="all" border="1" summary="General Python Naming Patterns"
       cellspacing="2" cellpadding="2">

  <tr>
    <th>Type</th>
    <th>Public</th>
    <th>Internal</th>
  </tr>

  <tr>
    <td>Packages</td>
    <td><code>lower_with_under</code></td>
    <td></td>
  </tr>

  <tr>
    <td>Modules</td>
    <td><code>lower_with_under</code></td>
    <td><code>_lower_with_under</code></td>
  </tr>

  <tr>
    <td>Classes</td>
    <td><code>CapWords</code></td>
    <td><code>_CapWords</code></td>
  </tr>

  <tr>
    <td>Exceptions</td>
    <td><code>CapWords</code></td>
    <td></td>
  </tr>

  <tr>
    <td>Functions</td>
    <td><code>lower_with_under()</code></td>
    <td><code>_lower_with_under()</code></td>
  </tr>

  <tr>
    <td>Global/Class Constants</td>
    <td><code>CAPS_WITH_UNDER</code></td>
    <td><code>_CAPS_WITH_UNDER</code></td>
  </tr>

  <tr>
    <td>Global/Class Variables</td>
    <td><code>lower_with_under</code></td>
    <td><code>_lower_with_under</code></td>
  </tr>

  <tr>
    <td>Instance Variables</td>
    <td><code>lower_with_under</code></td>
    <td><code>_lower_with_under</code> (protected)</td>
  </tr>

  <tr>
    <td>Method Names</td>
    <td><code>lower_with_under()</code></td>
    <td><code>_lower_with_under()</code> (protected)</td>
  </tr>

  <tr>
    <td>Function/Method Parameters</td>
    <td><code>lower_with_under</code></td>
    <td></td>
  </tr>

  <tr>
    <td>Local Variables</td>
    <td><code>lower_with_under</code></td>
    <td></td>
  </tr>

</table>

##### Names to Avoid
There are some names we want to avoid! These may be names that are claimed by Python, names that aren't helpful to other readers of your code, or names that are confusing.

Do not use:

* Single character names, except:
    * In iterators (ex, `for i in range(100):`)
    * `e` as an exception handler
    * `f` as a file object
* `__double_underscore_names__`, because these are reserved by Python!
* Names that needlessly include a type
* Names that are offensive (obviously, right?)
* Names that are meaningless (such as `cool_constant` or `fun_variable`)

##### Mathematical Names
Sometimes, we will use our code to implement common mathematics or algorithms. In this case, we may want to use short variable names. Here are some things to consider about that:

* Short mathematical names should be generally understood. For example, using `pi` to represent Pi and `v` to represent velocity is generally understood and acceptable. However, using `vd` to represent *velocity x distance* would not be acceptable, as this is not a clear, accepted term.
* If possible and it makes sense, try using a more descriptive name.
* Add a short line comment after the first use of the variable if it could help future readers. You may also desire to include units here as well.

##### File Naming
Files should end with `.py` and should not use dashes (`-`), but rather underscores (`_`). If you do not want the `.py` ending on the Python file and would prefer the file to take the role of an executable, consider making a symbolic link or a shell script wrapper that runs the Python file. (This can be as simple as `exec "$0.py" "$@"`!)

#### Imports
Imports are a powerful feature of Python. Here is a quick guide for using imports in Python:

```python
# Entire modules use
# import x
import datetime

# Specific modules from parent modules use
# from x import y
from dateutil import parser

# Long module names should also use "as" keyword
from example.package import super_long_subpackage_name as super_long

# Common packages can also use "as"
import numpy as np

# For relative imports use .
# For example:
# Assume this file is in /folder/subfolder where mod2 also lives
# Bad:
import mod2

# Good
from folder.subfolder import mod2
```

#### Exceptions and `assert`

Exceptions are a handy feature of Python that helps mitigate code that is breaking. `assert` is a helpful keyword in Python that allows one to test whether an internal statement is true, and is often used to test for internal correctness!

When attempting to catch exceptions:

* Do not catch `Exception`. Attempting to catch `Exception` will catch every exception thrown, which could indirectly catch exceptions that the program was not meant to catch. Instead, catch for a specific exception that will be raised. There is one case in which catching `Exception` is acceptable, however:
    * Attempting to catch all exceptions in attempt to mitigate exceptions blocking some code from running. For example, in order to handle exceptions quietly rather than ending a thread which runs a specific process.
* Keep `try` blocks to a minimum. As `try` blocks grow in size, the probability of *some exception* being raised increases, which may hide the true reasons behind some exceptions. Instead, attempt to keep logic outside of `try` blocks and instead only use `try` to catch code that could directly throw the exception you are trying to catch.
* Feel free to generate your own exception classes which inherit from built-in exception types. However, in many cases, it makes more sense to only use standard exceptions. For example, there is no need to create a new exception class for catching improper types passed into a method - instead, just use `TypeError`.

When using `assert`:

* Keep its use internal. For example, don't use `assert` to validate human input into a method. Instead, use it to verify to check types or the output of a helper method.

#### Iterators

Iterators provide a powerful way to loop through your code!

* When using list comprehensions, each part of the comprehension (`A for B in C`) should fit on one line. If the list comprehension has more than one `for`, then use a traditional iterator.
* Attempt to use default iterators when possible:
```python
# Yay!
for key in dict:
    ...

for k, v in dict.items():
    ...

# Noooo :(
for key in dict.keys():
    ...

for k, v in dict.iteritems():
    ...
```

#### Yielding

Sometimes, it may be desired to have your method **yield** values rather than simply **return** them. Yielding in Python is a powerful feature which delays your method's execution until you need it.

To have a method yield rather than simply return, use the `yield` keyword in the method and mark the method docstring with `Yields:`.

#### Lambda Functions

Lambda functions are mini-functions. They are expressed like so: `lambda x, y: x + y`.

If you use lambda functions:
* Keep them to one line. If they are longer than 80 characters, just use a nested function.
* Use them sparingly. Using complex, important operations in a lambda functions makes the code harder to debug and harder for other members to understand.

#### Conditional Expressions

Conditional expressions are short expressions which use the `if` and `else` keywords. Conditional expressions are powerful tools which can be undertood by new editors of the code.

Remember to keep the result, `if`, and `else` sections of the expression to only one line. For example:
```python
# Good
a = (b if c else d)
really_long = (evaluating_function(parameter)
               if really_long_variable_name
               else other_really_long_var_name
              )

# Bad
super_long = (evaluating_function(parameter)
              if (really_long_variable_name_one
              and this_iz_cool_func(cats))
              else 0
              )
```

#### Properties

Properties are a powerful feature of Python which allow traditional class methods to be hidden as attributes. Sneaky! This adds the benefit of masking getters and setters as traditional properties, but can accidentally mask complexion in the code that should not be hidden.

Properties should:
* Not be used for very simple operations (such as just returning a value)
* Not be used when the method invokes a lot of operations. The calling user may not understand this and accidentally invoke a lot of operations that block other processes.
* Always be marked with the `@property` decorator.

#### Implicit True/False

Python can evaluate a wide variety of statements to `True` and `False`. For example:
```python
a = []
if not a:
    # If a has no elements
    do_something()

b = ""
if b:
    # If len(b) > 1
    do_something()
```

In those statements, `True` and `False` were never explicitly used. However, they were implicitly used.

Attempt to use these statements when possible, as they help to make our code look more crafted and cute. However, keep some things in mind:
* If you are checking for `None`, just use `if x is None`. If you use `if not x`, then `x = False` will also trigger the conditional statement. This could be a problem when checking to see if a parameter is `None` - if the user passes in `False`, then you're going to have a little problem!
* Be wary when using these types of statements when checking the value of integers. Using something like `if not x / 2` is confusing because both boolean and numerical statements are involved.
* When using this type of statement to check the size of a Numpy array, use `if (not) array.size` instead of `if array`.

#### Decorators

**D**ecorator = **D**angerous! Sorta. Decorators are powerful for changing the behavior of methods, which can be helpful when operations in the method itself do not suffice.

However, decorators are confusing for new readers of the code, new users to Python, hard to recover from in the case of a raised error, and hard to debug. In the case that a decorator breaks a wrapped function, a MIL member may assume that the function which was operated on by the decorator was at fault, when this may not always be the case.

Therefore, when using decorators, keep in mind:
* Please test decorators extensively.
* Every decorator should be extensively documented.
* Please use decorators judiciously.

##### Line Length

Please keep lines to 80 characters or less. This can be seen in vim by using the option `colorcolumn=80`. If you have long strings, then use parentheses to implicitly connect multiple strings together:
```python
dr_seuss = ("Say! I like green eggs and ham!"
            "I like them! I do, Sam-I-Am!")
```

There are some exceptions:
* URLs in comments. Don't split these up.

#### Blank Line Separators

Use blank lines to separate different parts of your module. Use two lines to seprate top-level elements of any file (whether they are classes or methods) and one line to separate other distinct elements of the file.

#### Whitespace

Whitespace (not to be confused with greenspace, redspace, or rainbowspace!) can be incorrectly used in files to give the appearance of weird formatting.

When using whitespace:
* Do not use whitespace in front of a comma or colon.
* Always surround comparison operators (`==`, `!=`, `<`) with a whitespace character. 
* Do not put a whitespace character before a an index slice (such as `x [1]`) or function call (such as `fun (20)`).
* Do not include whitespace inside brackets including parentheses, brackets, or braces. For example, use `(x + 1)`, not `( x + 1 )` or `{'schwartz': True}`, not `{ 'schwartz': True }`.
* In function calls, do not use whitespace when passing a default parameter, unless a type annotation is present. For example, do not use `test(a, b: float=0.0)`, instead use `test(a, b: float = 0.0)`.

#### String Formatting

Many times you will want to format strings in a certain way: to add variables into the string, to make the string look pretty, or to use the string in a certain context, such as an enumerated iterator (`for i, v in enumerate(list): print(i + v)`).

In general: Just use f-strings! f-strings are a special type of string and can be made by prepending the first single/double quote of a string with `f`. This special string allows you to add in expressions into the string by using a pair of curly braces:
```python
ultimate_answer = 42
print(f"The ultimate answer is {ultimate_answer}.")

print(f"The ultimate answer is not {ultimate_answer // 2}!")
```

When formatting a string that will be printed for logging/debugging, attempt to use a common starting term such that the string can be found with a search tool, such as `grep` or the terminal's search functionality. Additionally, make clear what sections of the logged method are interpolated or calculated.
```python
# Please no!
import random
rand = random.random()
print(f"The {rand} number is less than 1.")

# Now, you can search for "This number is less than one" and find all matching instances!
print(f"This number is less than one: {rand}")
```

#### TODO Comments

TODO comments are a great way to mark a piece of code as not currently finished. To use TODO comments, simply create a comment that starts with `TODO`, has your name, and what still needs to be done. Simple!
```python
# TODO (Dr. Schwartz) - Finish method to add c in final sum
def sum(a: int, b: int, c: int):
    return a + b
```

#### Getters and Setters

Getters and setters serve as dedicated methods for getting and setting a property of a class. These are similar to `@property` methods, but these are explicit methods which are not hidden as properties.

Getters and setters should only be used when changing a single property causes significant overhead or recalculation of other properties. Otherwise, simply set the property as public or use a small `@property` method.

Getters and setters should be named in a way that clearly demonstrates which property is being set. For example, `get_weather()` and `set_weather()`.

#### Function Length

A function should roughly be **30 lines**. This is not a hard rule, but is a general rule to keep in mind when writing and reading functions. If you see a function longer than this, feel free to break it up if it makes sense.

### Typing
**Please, whenever possible, add type annotations to your code.** Typing is a powerful feature of Python that helps others read and understand your code.

Type annotations take on the following format:
```python
def sum(a: int, b: int) -> int:
    return a + b
```

The `int` keyword after the `a` and `b` parameters of the method signals that the method accepts two integers. The `-> int` at the end of the function signature signals that the method then returns an `int`.

Other common built-in type annotations you might see include `float`, `dict`, `bool`, or `None`. Sometimes, though type annotations can be a little more complex. How do you write the type of a parameter that is optional? Or how you do you write the type annotation of a method that can return a `List` or a `dict`? Or how about a method that returns another method?? (Method inception!)

This is where the built-in `typing` module comes in. This handly module contains all sorts of classes to represent types throughout your code. Let's have a look.

```python
from typing import List, Union, Optional, Callable

def crazy_func(a: int, b: Union[List[float], dict], c: Optional[dict] = None) -> Callable:
    # Do some crazy stuff
    return d
```

What the heck is that method even saying?
1. First, `a` accepts type `int`.
2. Second, `b` accepts either (this is what the `Union` class represents) a list of `float`s or a `dict`.
3. `c` *can* accept a `dict`, but it doesn't have to - supplying this parameter is `Optional`.
4. The function returns another method, known to `typing` as a `Callable`.

Note that if your type annotations are overly long, it may be a good idea to split up your method into a series of shorter methods.

### Docstrings

### Linting & CI
Explain the process of Python linting, what is used, and what happens to one's code upon the linter striking it.

### Other Tools
Link to some other helpful tools that members can use to make formatting and checking easier.
