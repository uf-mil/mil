# Developer Tools

If you're interested in sticking around in MIL for a while (as we hope you are!),
we recommend installing some tools that will make your work a little easier.
These tools serve to be like little assistants, polietly cleaning up your code
behind you and warning you about potential errors that may occur as you change
existing code and contribute new code.

Let's discuss how we can use some of these tools.

## `pre-commit`

One tool that most developers (if not all) should have installed is `pre-commit`.
This command-line tool checks your code for linting errors, spell check errors,
unneeded whitespace, and the formatting of your files. And, it does this
automatically, without you even needing to ask! How awesome is that?

Basically, `pre-commit` runs on a shared config file that all MIL members use.
This config file specifies what the tool should check for, such as if you have
extra whitespace at the end of the line. When you attempt to commit changes to
the repo, the tool will block your commit if it finds any errors. Some of the
hooks will automatically format the code into the proper way for you, while
others will show you what part you need to change yourself.

Furthermore, our CI checks use the same hooks specified in the config file. This
means that if `pre-commit` does not pass on your local computer, it's not going
to pass when you try to merge it into the robot's codebase. Therefore, it's
highly recommended to install the tool to save yourself some time in the long run.

### Installing

To install `pre-commit`, use `pip`! We specify the version of `pre-commit` in
a requirements file, which is why you reference it in the command below.

```sh
$ pip3 install -r requirements_test.txt
```

After doing this step, you should be able to run the `pre-commit` command. Now,
we will install the hooks specified in our config file. This command will also
install a git hook into your git configuration. This allows the tool to run right
before you attempt to commit a piece of code.

```sh
$ pre-commit install
```

Now, try to run it against all of our files! It should show that all tests have
passed. Some may have been skipped.

```sh
$ pre-commit run --all-files
```

If you have any trouble with this tool, feel free to check out
[the pre-commit website](https://pre-commit.com).

## Vim

Another helpful developer tool is having a good Vim configuration, if that's the
editor you choose to use (the most popular editor amongst MIL members).
A good Vim configuration can provide helpful code suggestions and diagnostic
features, helping you to catch errors write after you write a particular line
of software.

We'll see how we can set up a helpful Vim configuration below.

### Introducing Vim

What is Vim, anyways, and why do many MIL members use it? Vim is a code editor
that runs inside your terminal. You may know of other editors such as VSCode
or PyCharm or CLion, but these are GUI programs. You can click around them and
press a bunch of buttons to do things.

In Vim, there are no buttons, and usually, no clicking. You only use your keyboard
to do everything you want (to move, to change settings, delete lines, etc.).
How? A bunch of keyboard shortcuts, and a "Vim language" for coordintaing movement!
These settings make Vim super flexible, allowing you to bring the software generally
anywhere.

There are a lot of great Vim tutorials around the Web, so we won't discuss many
of the keyboard shortcuts here. If you're interested in learning the Vim way,
feel free to search for a book, game, or guide that walks you through the editor.
Because many MIL software leaders use Vim or have used Vim before, we are here
to help you out as well!

### Installing Neovim

A popular fork of the Vim project is a piece of software named Neovim. This
project adds more functionality to Vim, allowing for more flexibility in
extensions and modifications.

You can install Neovim through:

```sh
$ sudo apt-get install neovim
```

### Installing a distribution

One method for getting a nice Neovim setup going involves installing a particular
distribution of the software. These distributions come with a pre-configured
set of extensions and configuration files that make setting up a nice helpful
Neovim configuration painless!

Some popular Neovim distributions include:
* [SpaceVim](https://github.com/SpaceVim/SpaceVim)
* [LunarVim](https://github.com/LunarVim/LunarVim)
* [spf13's Vim configuration](https://github.com/spf13/spf13-vim)
* [NvChad](https://github.com/NvChad/NvChad)

If you choose to setup one of these distributions, you don't need to read
the rest of the tutorial, and you'll have nice features in your own Vim configuration!
That's really cool, which makes this a great option! However, if you'd like to setup
your own configuration, keep reading.

### Writing a configuration

You may be inclined to write your own configuration for Neovim! That's great.
Chances are if you do this, you'll be able to learn a lot about Vim and Neovim
in the process. However, it's also a lot. Because there's so much to it, we won't
walkthrough the process here. However, here are some things you may want to setup:

* Installing an extension manager ([vim-plug](https://github.com/junegunn/vim-plug) for example!)
* Setting up a sane escape key (such as `jk`)
* Setting up integrated LSP (Try checking out [nvim-lspconfig](https://github.com/neovim/nvim-lspconfig)!)
* Setting up diagnostics and formatters used in `pre-commit` with [null-ls.nvim](https://github.com/jose-elias-alvarez/null-ls.nvim)
* Finding a beautiful colorscheme!
* Setting up extensions to make your life easier (A lot of great ones can be found at [vimawesome](https://vimawesome.com/))
