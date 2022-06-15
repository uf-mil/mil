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

A good practice is to run `black` (and `python-modernize` if you find that it helps
you) before you begin the process of migration. Then, re-run the command(s) again
before you commit.

### Step Two: Migrating the code

Great, the code is now pretty! :D

If you ran `python-modernize`, it will have suggested some changes to you, indcated
by `+` and `-` signs. It may look something like this:

```diff
RefactoringTool: Refactored old.py
--- old.py	(original)
+++ old.py	(refactored)
@@ -1 +1,2 @@
-print 'this is an example'
+from __future__ import print_function
+print('this is an example')
RefactoringTool: Files that need to be modified:
```

This is showing that it's suggesting the removal of the `print 'this is an example'` line,
the addition of `from __future__ import print_function`, and the addition of
`print('this is an example')`.

Let's break this down:
1. `python-modernize` frequently likes to suggest adding `from __future__ import X`.
The good news is that most of the time, this isn't needed at all. The only exception
is `from __future__ import annotations`, which you may need to add if it suggests adding it.
1. It's suggesting the replacement of `print '...'` with `print('...')`, which is
an awesome idea! You may have noticed this is a required change in the Python 2 to 3
Porting Guide above.

Great! So now we know what to fix in our file. Here's some more tips about `python-modernize`:
1. It always starts with a long list of "fixers" telling you what it looked for
in the file. This is okay; you just need to ignore the list.
1. Some changes ask you to use `import six.XXX`. Never do this! This is covered more below.
1. After migrating `print '...'` to `print('...')`, you may notice that `python-modernize`
wants you to now write `print(('...'))`! Don't do this - only one set of parentheses
is needed.
1. Usually, a method beginning with `iter` can have the `iter` prefix removed. For example,
`for k, v in dict.iteritems()` can be migrated to `for k, v in dict.items()`.

Additionally, change the [shebang](https://en.wikipedia.org/wiki/Shebang_(Unix)) of the file if that was not a suggested fix by `python-modernize`. You likely only need to replace the `python` with `python3`!

#### Handling `import six.XXX` suggestions
`six` is a Python 2 to 3 library used to help in the transition between the two
languages by providing some helper functions and methods that work in both languages.

When the program asks you to import something from `six`, it's asking you to prop
up our code on these crutches. But, we are MIL, and we don't need any crutches!
Instead of using `six`, try fixing the root problem. For example:

```diff
RefactoringTool: Refactored old.py
--- old.py	(original)
+++ old.py	(refactored)
@@ -1 +1,2 @@
-test = range(2)
+from six.moves import range
+test = list(range(2))
RefactoringTool: Files that need to be modified:
RefactoringTool: old.py
```

It's asking us to import `range` from `six.moves`. It's telling us that our `range`
function is messed up, and that we need to do something about it. You'll also notice
that it wants us to wrap the `range` in a `list`.

The porting guide linked above can come in real handy here. It explains that in Python 2,
`range` always produced a `list`, while in Python 3, it produces a `range` object.
Because we want the same functionality as we had in Python 2, we'll need to manually
convert the `range` object to a `list`. Once that's done:

```diff
RefactoringTool: Refactored old.py
--- old.py	(original)
+++ old.py	(refactored)
@@ -1 +1,2 @@
+from six.moves import range
 test = list(range(2))
RefactoringTool: Files that need to be modified:
RefactoringTool: old.py
```

Now the suggestion of wrapping `range` in a `list` is gone, but it's still
asking us to `import range`. We can just ignore this, because we know that we fixed
the root problem. Great job!

### Step Three: Documenting the code (optional!)

An optional step in the migration process is documenting the code as you convert it. 
Documenting the code helps future members understand the code they are reading.

Documentation comes in many forms! One form is by typing the code, a feature of 
Python that Python 3 supports. To type the code, add the types that the method 
accepts as parameters and what types it returns.

Another form is by adding docstrings throughout the code, which help to explain 
what methods do and how they work. These docstrings can be added through a 
multi-line comment under the method signature. The docstring should include the 
type and an explanation for each argument and what the method returns.

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
def postal_valid(s: str) -> bool:
    """
    Returns whether a postal code is valid.

    Args:
        s (str): The postal code, represented as a string.

    Returns:
        bool: Whether the code is valid.
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

Now that you've added new documentation, let's see it built:
```sh
$ mil
$ ./scripts/build_docs
```

That last command should build the documentation on your computer and provide you
with a link - clicking that link should open up an HTML page where you can see
your beautiful new documentation! How exciting!

When a new member sees the method in the first code block, how are they supposed 
to know what it does? This is what the docstring and typing annotations help with! 
Now, another member can instantly see the types of parameters the method accepts, 
what it returns, and what it is supposed to do.

Again, this is totally optional. But, if you complete this step, you will have 
the opportunity to learn much more about the codebase and how each module works!

### Step Four: Check and verify

Great! By now, the code should be ready to be run in Python 3. For a last step check, 
run `python-modernize` again and verify that any warnings that appear do not need 
to be fixed. Finally, run `black` again.

## Updating CMake minimum verison

The CMake minimum version in each package needs to be updated to version `3.0.3`. 
This has already been completed for all packages and should not need to be completed again.

## Testing the changes

After making some changes to a package, you may have the desire to test the changes that you've made. But wait! You can't.

Many of the packages in our codebase rely on each other like an interlocking Jenga tower. If one block is removed, the tower will frequently fall. Therefore, if you attempt to simulate your changes after making some changes, the simlulation may just fail.

Therefore, make the best changes that you can; changes that you feel confident in. Once all packages have been updated for ROS Noetic, then we can test to see how our all the packages perform. If things are still broken at this point, then we can make further commits to fix what's broken.
