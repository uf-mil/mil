# Updating Documentation
So, you'd like to update this documentation? That's fantastic! Documentation will help
countless numbers of MIL generations to understand what you've built. If you only build
a system, but don't document it, future generations may not even know what to do with
what you've built!

:::{warning}
For members who do not have ROS setup (most electrical and mechanical members),
you can still contribute to the documentation, but you will not be able to add
reference documentation (ie, docstrings).

Furthermore, you will need to build the documentation differently. See the
[Viewing changes](viewing_changes) section below for more information.
:::

## ReStructured Text

One of the most common formats you'll see used for documentation is ReStructured
Text, or RST. These files usually end in `.rst`. Furthermore, all docstrings
are automatically rendered using RST.

To create and edit documentation, you don't need to know too much RST. For a nice
introduction, check out [this primer](https://www.sphinx-doc.org/en/master/usage/restructuredtext/basics.html).

If you're writing new documentation, feel free to write it in RST, or the alternate
MyST, described below.

## Markdown and MyST

While RST is great (and used by default), its syntax can sometimes feel dated and
old. Therefore, a replacement was developed: MyST. Similar in name, this alternate
parser allows you to write documentation using Markdown rather than RST.

To use MyST instead of RST, write documentation files in Markdown instead of RST!
Using a `.md` extension should be enough to tell Sphinx which parser to use.

Everything that's available through the RST syntax can be found in MyST - the syntax
is a little different. For more information on MyST and how to use it, check out
its [documentation](https://myst-parser.readthedocs.io/en/latest/index.html).

## Admonitions

Sometimes, you want to make things a little more evident. Admonitions are great
for this purpose. Admonitions are the "Danger" or "Note" boxes that you see
around the docs.

They are compatible with both RST and MyST and support a variety of syntax:

```rst
.. danger::

    It's dangerous to go alone! Take this admonition.
```

```md
:::{warning}
Oh my gosh! Be careful.
:::
```

## Docstrings vs. Explicit Documentation
There are two main types of document in our repository: docstrings and explicit
documentation.

Docstrings are documentation **within** the code itself. These docstrings are then
found by Sphinx and compiled into the repository reference documentation.

Explicit documentation is documentation created by members by using files under
the `docs/` folder. Explicit documentation is not connected to any code we use
in particular, which makes it very flexible.

## Docstrings
Docstrings are a powerful method of documentation. The documentation is outlined
in the code, allowing developers to grasp what classes and methods do as they're working
on them. Plus, these docstrings get concatenated together to form the HTML reference
docs. (Hint: These are what make up most of the [Software Reference page](/reference/index.rst)!)

### Adding a docstring (Python)
To add a new docstring to the reference documentation, add a docstring inside of the code.

Let's look at an example:

```python
def download_and_unzip(url, output_dir):
    try:
        html = download(url)
    except:
        raise IOError("Could not load file at {}".format(url))

    fake_file = StringIO.StringIO(html)

    zip_ = zipfile.ZipFile(fake_file, "r")
    for file_path in zip_.namelist():
        _, file_name = os.path.split(file_path)
        file_like = zip_.open(file_path)

        f = open(os.path.join(output_dir, file_name), "w")
        f.write(file_like.read().decode('utf-8'))
        f.close()
```

Can you tell what this method does? You could maybe look at some of the lines and
guess as to what it's doing - but this isn't ideal. Without a docstring, this is
how you (and all of the other software MILers) have to understand this code! Let's
fix that by adding a docstring.

```python
def download_and_unzip(url: str, output_dir: str) -> None:
    """
    Downloads a zip file at a particular URL and unzips it to a directory.

    Args:
        url (str): The URL to obtain the zip file from.
        output_dir (str): The location of where to write the zip contents to.

    Raises:
        IOError: The file at the URL could not be found/loaded.
    """
    try:
        html = download(url)
    except:
        raise IOError("Could not load file at {}".format(url))

    fake_file = StringIO.StringIO(html)

    zip_ = zipfile.ZipFile(fake_file, "r")
    for file_path in zip_.namelist():
        _, file_name = os.path.split(file_path)
        file_like = zip_.open(file_path)

        f = open(os.path.join(output_dir, file_name), "w")
        f.write(file_like.read().decode('utf-8'))
        f.close()
```

Wow! Look how much clearer that is. You know what the type of each argument is, and
what it represents. You can see any errors that the function might raise, as well, along
with what it returns (`None`). And, this is available in the code and on the docs website!

So, you're all good, right? Not yet! You need to make sure that the function's
docstring will be shown on the reference page. In `reference.rst`, you need to
add either an `.. autofunction:: ` or `.. autoclass:: ` directive. (Check out
the document to see examples!)

Then, when you build the docs, the docstring will be added to the docs. For more information
on docstrings in Python, consult the [Python style guide](/software/python_style).

### Adding a docstring (C++)
To add new docstrings for C++ code, you should add the docstrings in the necessary
header files.

Find the appropriate function or class, and add a docstring comment before its
declaration. You can use JavaDoc-like syntax to add annotations.

For example:

```cpp
/**
 * Waits for a connection to be established on the heartbeat listener. If no connection
 * is established before the timeout has run out, then false is returned and the
 * function exits.
 *
 * @param timeout The amount of time to wait before exiting the function and returning
 * false.
 *
 * @return Whether a connection was established in time.
 */
bool waitForConnection(ros::Duration timeout = { -1.0 }) const;  // waits forever by default
```

This docstring includes a general description of the function, along with what parameter
it's poised to accept. Additionally, what the function returns is documented. Great!

## Explicit Documentation

### Creating the file
Create the file in the `docs/` directory of the repository, and place the document
under a folder which makes sense (ie, `mechanical/` for mechanical documentation).
You can use either reStructuredText or Markdown for creating your documents, but if
you are making an index or navigation-focused page, use reST.

### Indexing the file
For your newly added page to be found, it needs to be added to a table of contents. There is a root table of contents in `index.rst` and smaller table of contents within various subdirectories. Add your page to one or both of these, whichever you deem more appropriate. To do so, add the path to your page (without the extension) under the
`.. toctree::` section of the `index.rst` file. For example, to add the "Meeting Schedule" page mentioned above, the `index.rst` will now look like

```rst
.. toctree::
   :maxdepth: 1

   my_article
   Meeting Schedule <meeting_schedule.md>
```

Sometimes, though, you don't want your document to be in a `toctree`. In this case,
add it to a `:hidden:` TOC tree, like so:

```rst
.. toctree::
   :hidden:

   peek_a_boo
```

This TOC tree won't be rendered, but Sphinx will still allow you to build the docs
because the document is in some TOC tree.

<!-- Reference in MYST -->
(viewing_changes)=

## Viewing changes
It is important to generate the documentation website locally and view it in a
web browser to verify your changes did what you wanted them to do and they are
quick to read.

For members who have ROS setup, you can build the documentation with the following
commands:

```bash
$ mil
$ ./scripts/build_docs
```

Members without ROS setup will need to build the documentation without the source
files. Because you aren't building the reference files, you will get a few errors
explaining that Sphinx can't find those files. This is expected. Errors related
to files you've modified may be a result of your changes, however.

```bash
$ mil
$ ./scripts/build_docs --no-source
```

At the bottom of each build output, a link is displayed where you can access
the built documentation. Keep in mind that this link does not start with `https://`,
but rather `file://`.

After you've built the repository for the first time, the build contents are cached,
meaning that subsequent changes the docs will only re-build the files you edited
since you last built the docs. If this is not ideal behavior, you can completely
re-build the docs by adding another flag:
```bash
$ mil
$ ./scripts/build_docs --scratch
```

## Contributing changes
Now that you have made and verifed your changes, follow the [contributing guide](contributing) to add your changes to the repository.
