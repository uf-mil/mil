# Working with Git Submodules

Submodules are an important part of the MIL repo! These submodules allow our code to function properly! Formally, these submodules are just directories inside of our repository somewhere which are actually a Git repository themselves. In essence, these directories (inside of the main MIL directory) are pointing at a specific commit inside of another Git repository.

This pattern can be confusing, so here are some commands to help you out!

## Adding a new submodule

To add a new submodule into a repository, you can use `git submodule add <url>`!

```sh
$ git submodule add https://github.com/chaconinc/DbConnector
```

When you do this, a new directory called `DbConnector` will appear, and the `.gitmodules` file (which keeps track of your submodules) will be updated. Now, you'll need to actually commit it to the repository, just like any other folder you add:
```sh
$ git commit -am 'Add DbConnector module'
$ git push
```

## Cloning a repository with submodules

When you clone a repository with nested submodules, you must run some commands to get your submodules up and running:
```sh
$ git submodule update --init --recursive
```
In this command, `--init` initializes your submodule configuration file, while `--recursive` does this for all nested submodules in the project.

## Pulling in upstream changes in submodules
### Through the submodule remote

Sometimes, submodules can be updated by another user, and you'll need to pull these changes into your own copy. If you would like to do this for just one submodule, `cd` into the submodule, and then update the specific submodule yourself:

```sh
$ cd DbConnector
$ git fetch
$ git merge origin/master
$ cd ..
$ git commit -am 'Pulled in new submodule code'
$ git push
```

Because you committed with the new submodule code, whenever others fetch new changes, they will also get the update submodule changes.

While the above is great for small, independent submodule changes where you want more customizability in how the submodule is updated, repeatedly doing multiple commands just to update the submodule can get annoying! Thankfully, git helps to fix that with:
```sh
$ git submodule update --remote
```
This command will update all submodules from their `remote` repositories. If you want to update a submodule from another branch its on, you can change your Git config.
```sh
$ git config .gitmodules submodule.DbConnector.branch stable
$ git submodule update --remote DbConnector
```
Now, when you update the `DbConnector` repository, it will pull from the `stable` branch. You can then `git commit` and `git push` as normal.

### Through the main project remote

Normally, to update the project, you would run `git pull`. However, when working with submodules, this won't be enough to create effective changes. This will get the relevant changes that occurred in the submodules, but it won't actually *update* the submodules.

Instead, you will also need to run:
```sh
$ git submodule update --init --recursive
```

These two commands can be simplified into one through:
```sh
$ git pull --recurse-submodules
```

Sounds like a lot to type? Let's make `--recurse-submodules` default!
```sh
$ git config submodule.recurse true
```

## Working on a submodule
Eventually, you may want to make some changes to the submodule to help improve the code!

Before working, let's merge in any changes from the upstream:
```sh
$ git submodule update --remote --merge
```

Now, we can make some changes:
```sh
$ cd DbConnector/
$ vim src/example.cpp
$ git commit -am "Add example test"
```

You've now made a commit to the submodule. If it's been a while and you want to rebase changes from the upstream:
```sh
$ git submodule update --remote --rebase
# OR
$ git submodule update --remote --merge
```

But, we still haven't actually pushed the changes!
```sh
$ git push --recurse-submodules=on-demand
```
Note that this will push any commits for **every** submodule. If you just want to push some commits for one submodule, then `cd` into the submodule directory and use `git push`.
