## Python Style Guide
Welcome to the Python style guide for MIL. This guide is meant to be a relaxed, easy-to-follow guide with some tips on how to make your Python look snazzy! It's based on the [ROS style guide](http://wiki.ros.org/PyStyleGuide) and the [Google Python style guide](https://google.github.io/styleguide/pyguide.html).

Don't feel pressured to know or understand every rule, and totally feel free to skim over the guide at first. This guide can provide some helpful information before you begin to write your Python, but it also serves as an excellent place to determine how to use a particular Python feature in your code.

Additionally, this guide is not meant to be permanent! If you have a suggestion, feel free to bring it up and change the guide! Have fun!

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

### Linting & CI
Explain the process of Python linting, what is used, and what happens to one's code upon the linter striking it.

### Other Tools
Link to some other helpful tools that members can use to make formatting and checking easier.