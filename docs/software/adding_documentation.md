# Adding/Modifying Documentation
We aim to have all MIL documentation (software, mechanical, electrical, etc) hosted 
within the main [MIL git repository](https://github.com/uf-mil/mil). Please 
follow these instructions to add new documentation or update/correct existing 
documentation.

## Docstrings vs. Explicit Documentation
There are two main types of document in our repository: docstrings and explicit 
documentation.

Docstrings are documentation **within** the code itself. These docstrings are then
found by Sphinx and compiled into the repository reference documentation. Explicit
documentation is documentation created by members by using files under the `docs/`
folder.

## Docstrings

### Adding a new docstring
To add a new docstring to the reference documentation, add a docstring inside of the code.

Let's look at an example:

    # From: https://stackoverflow.com/a/11832677

    import re
    def postal_valid(s):
        spaceless = s.replace(' ','')
        if not re.match(r"[a-zA-Z][0-9]+[a-zA-Z][0-9]+[a-zA-Z][0-9]+",spaceless):
           return False
        return spaceless.upper() 

Hmm, this method could use a docstring! To add a docstring to a Python object, use
a block string. For more information, check out the Python Style Guide.

    # From: https://stackoverflow.com/a/11832677

    import re
    def postal_valid(s: str) -> bool:
        """
        Validates a postal code.

        Args:
            s (str): The zip code to be validated.

        Returns:
            bool: Whether the postal code is valid.
        """
        spaceless = s.replace(' ','')
        if not re.match(r"[a-zA-Z][0-9]+[a-zA-Z][0-9]+[a-zA-Z][0-9]+",spaceless):
           return False
        return spaceless.upper() 

So, you're all good, right? Not yet! You need to make sure that the function's
docstring will be shown on the reference page. In `reference.rst`, you need to
add either an `.. autofunction:: ` or `.. autoclass:: ` directive. (Check out
the document to see examples!)

Then, when you build the docs, the docstring will be added to the docs.

## Explicit Documentation

### Creating the file
Create the file in the `docs/` directory of the repository, and place the document
under a folder which makes sense (ie, `mechanical/` for mechanical documentation).
You can use either reStructuredText or Markdown for creating your documents, but if
you are making an index or navigation-focused page, use reST.

### Indexing the file
For your newly added page to be found, it needs to be added to a table of contents. There is a root table of contents in `index.rst` and smaller table of contents within various subdirectories. Add your page to one or both of these, whichever you deem more appropriate. To do so, add the path to your page (without the extention) under the
`.. toctree::` section of the `index.rst` file. For example, to add the "Meeting Schedule" page mentioned above, the `index.rst` will now look like
```
.. toctree::
   :maxdepth: 1

   docs/my_article
   Meeting Schedule <docs/meeting_schedule.md>
```

Sometimes, though, you don't want your document to be in a `toctree`. In this case,
add it to a `:hidden:` TOC tree, like so:
```
.. toctree::
   :hidden:

   docs/peek_a_boo
```

This TOC tree won't be rendered, but Sphinx will still allow you to build the docs
because the document is in some TOC tree.

## Viewing changes
It is important to generate the documentation website locally and view it in a web browser to verify your changes did what you wanted them to do and they are easy to read.

    $ mil
    $ ./scripts/build_docs
    $ ./scripts/display_docs

## Contributing changes
Now that you have made and verifed your changes, follow the [contributing guide](contributing) to add your changes to the repository.
