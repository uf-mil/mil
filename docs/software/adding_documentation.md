# Updating Documentation
So, you'd like to update this documentation? That's fantastic! Documentation will help
countless numbers of MIL generations to understand what you've built. If you only build
a system, but don't document it, future generations may not even know what to do with
what you've built!

## Setup for Members with ROS

If you have ROS installed, that's great. You will be able to build source files
from our source code. Make sure you have completed the getting started process
to ensure you have all the needed dependencies.

Remember to make your changes on an accessible branch:
```bash
$ git checkout -b <your branch name here>
```

Feel free to be flexible with this; you may not need to make a new branch if you
already have one that you'd like to add documentation to.

## Setup for Members without ROS

We want to make the MIL docs accessible to all members, including those who
may not have ROS setup. Below are the steps you will need to take to get a _lite_
setup working that allows you to contribute to the docs and preview your changes.

1.  Install git and clone the repository.
    ```bash
    $ git clone https://github.com/uf-mil/mil -j8 --recurse-submodules
    ```
2.  Make your own branch off of `master`. This branch only will be used by you
    (and any collaborators working with you), so feel free to play around with things.
    ```bash
    $ cd mil
    $ git checkout -b <your branch name here>
    ```
3.  Install Python and the needed dependencies. This will vary based on your
    operating system.
    ```bash
    $ pip3 install -r requirements.txt
    ```
4.  Make changes to the files.
5.  To view your changes and contribute them, continue reading below.

## Updating styling

Most of the colors used on the documentation site are controlled through `custom.css`
or `codeblocks.css`. To update the general layout of the site, edit the `custom.css`
and re-build the documentation. If you haven't changed any actual content, you may
need to build the documentation from scratch using the `-s` flag.

To update the colors shown in code blocks, edit the `codeblocks.css` file. You can
either edit this file directly, or generate a similar copy by using the `pygments`
library. Navigate into the `uf_mil_pygments` Python package in the `docs/` folder,
install the Python package (`pip install -e .`) and then create a CSS file using
the Pygments Style class found in the package (`pygmentize -S mil -f html -a .highlight > mil.css`).

## Making changes
To make changes to the documentation, you only need to add/edit files under
the `docs/` folder or docstrings existing in the source code. Once you edit these
sources, you have changed the documentation, and this will be reflected in the
HTML you build.

To see the syntax used to make the documentation, check out the page on [syntax](/software/documentation_syntax.md).

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
Now that you have made and verified your changes, follow the [contributing guide](contributing)
to add your changes to the repository.
