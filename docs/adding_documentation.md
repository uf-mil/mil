# Adding/Modifying Documentation
We aim to have all MIL documentation (software, mechanical, electrical, etc) hosted within the main [MIL git repository](https://github.com/uf-mil/mil). Please follow these instructions to add new documentation or update/correct existing documentation.


## Deciding where to put the file
If you are creating a new page, please observe the following convention:
* If there is already a directory in the repository containing code/docs for the project you are documentating, put it there. For example, if you are writing a page "Tuning the controller", put it under `gnc/my_controller/tunning.md`
* For more general documentation put it under `docs/`. For example, a page on "Meeting Schedule" should go in `docs/meeting_schedule.md`

## Creating the file
Checkout some examples for how to format the documention under [documentation examples](examples/index)

## Indexing the file
For your newly added page to be found, it needs to be added to the table of contents. There is a root table of contents in `index.rst` and smaller table of contents within various subdirectories. Add your page to one or both of these, whichever you deem more appropriate. To do so, add the path to your page (without the extention) under the
`.. toctree::` section of the `index.rst` file. For example, to add the "Meeting Schedule" page mentioned above, the `index.rst` will now look like
```
.. toctree::
   :maxdepth: 1

   docs/my_article
   Meeting Schedule <docs/meeting_schedule.md>
```

## Viewing changes
It is important to generate the documentation website locally and view it in a web browser to verify your changes did what you wanted them to do and they are easy to read.

**Linux**

* Generate documentation `./ci/build_docs`
* Open the site `firefox /tmp/docs/html/index.html`

**Windows**

TODO

## Contributing changes
Now that you have made and verifed your changes, follow the [contributing guide](development/contributing) to add your changes to the repository.
