# Noetic Migration

Welcome! Thanks for helping with the migration to ROS Noetic. This guide will attempt to walk you through some of the tools we will use and some of the steps we will take to achieve ROS Noetic compatibility.

## Overview

**So, what even is the problem here? What is "ROS Melodic" and "ROS Noetic", and why do we need to migrate between them?**

First, ROS is the Robot Operating System, which controls how our robots perform. Released in 2018, ROS Melodic is the version our code currently operates on. ROS Noetic was released in 2020 and is the next major version of ROS after ROS Melodic.

Like most software, ROS Noetic fixes a lot of bugs and adds helpful features that can be used when programming our robots. Therefore, many systems are migrating over to ROS Noetic, including the VRX competition, one of the main competitions we compete in. To ensure compability with ROS Noetic in the future, we want to start migrating earlier rather than later so that we aren't left behind and can use the latest-and-greatest features on our robots!

So the migration between ROS Melodic is easy-peasy, right? A few setup scripts here, a touch of code editing there? Well, not exactly. There are a few major components of our systems that must be updated to work with ROS Noetic. This includes:

* Updating all **Python 2** to be compatible with **Python 3**
* Updating ROS packages to use a different version of CMake
* Verifying dependency compatibility with ROS Noetic
* Updating some dependencies, such as OpenCV and PCL
* *[Other various changes...](http://wiki.ros.org/noetic/Migration)*

We will work through these changes piece by piece to make sure our systems are up to date.

Let's start with the first major step...

## Migrating Python 2 to Python 3

Python is one of the most common languages found in our codebase. However, the code was written in Python 2, and it now needs to be converted to Python 3, which has a very different syntax style.

Before you start, take a **quick look** over [this porting guide](https://portingguide.readthedocs.io/en/latest/index.html). Start at the `Syntax Changes` section and skim through the end. You don't need to memorize this information, but skimming through the pages will give you a good idea for what changes need to be made.

Then, you can choose a package to convert to Python 3. To do so, go to the GitHub repository and choose and issue with the `2to3` label. Once you've chosen a package to migrate, you can go through a series of steps with each package:

### Step One: Running scripts against the existing code

First, run some helper scripts against the code that already exists in the package. There are some great tools that can help you with this:

* `python-modernize`: A command-line utility that is able to show what code needs to be migrated to Python 3. The utility will show what lines could cause issues if run with Python 3. Remember that a lot of Python is compatible with both versions 2 and 3, only some of it is not. Also note that not everything it flags is an issue, but more on this later.
* `black`: Another command-line utility useful for making Python look pretty and sweet! It formats the code in such a way that the meaning is preserved but the code is easier to read!

A good practice is to run these two commands before committing, if the `black` command made a significant amount of changes. This way, your changes are separated from the automated changes of the formatter.

### Step Two: Migrating the code

Great, the code is now pretty! :D

Look over the changes `python-modernize` suggested, and begin to implement suggestions. For more help with this, look at the section on `python-modernize` below.

Additionally, change the [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)) of the file if that was not a suggested fix by `python-modernize`. You likely only need to replace the `python` with `python3`!

### Step Three: Documenting the code (optional!)

An optional step in the migration process is documenting the code as you convert it. Documenting the code helps future members understand the code they are reading.

Documentation comes in many forms! One form is by typing the code, a feature of Python that Python 3 supports. To type the code, add the types that the method accepts as parameters and what types it returns.

Another form is by adding docstrings throughout the code, which help to explain what methods do and how they work. These docstrings can be added through a multi-line comment under the method signature. The docstring should include the type and an explanation for each argument and what the method returns.

For example, the following code block can be annotated with types and docstrings:

```python
# From https://stackoverflow.com/a/11832677
def postal_valid(s):
    no_spaces = s.replace(" ","")
    if len(no_spaces ) > 6: return false
    for i in range(6):
        if i%2:
           if not no_spaces[i].isdigit():return False
        else:
           if not no_spaces[i].isalpha():return False

     return no_spaces.upper() #True
```
to
```python
# From https://stackoverflow.com/a/11832677
def postal_valid(s: string) -> bool:
    """
    Returns whether a postal code is valid.

    Args:
        s: string - The postal code, represented as a string.

    Returns:
        bool - Whether the code is valid.
    """
    no_spaces = s.replace(" ","")
    if len(no_spaces ) > 6: return false
    for i in range(6):
        if i%2:
           if not no_spaces[i].isdigit():return False
        else:
           if not no_spaces[i].isalpha():return False

     return no_spaces.upper() #True
```

When a new member sees the method in the first code block, how are they supposed to know what it does? This is what the docstring and typing annotations help with! Now, another member can instantly see the types of parameters the method accepts, what it returns, and what it is supposed to do.

Again, this is totally optional. But, if you complete this step, you will have the opportunity to learn much more about the codebase and how each module works!

### Step Four: Check and verify

Great! By now, the code should be ready to be run in Python 3. For a last step check, run `python-modernize` again and verify that any warnings that appear do not need to be fixed. Finally, run `black` again.
