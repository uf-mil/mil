# Contributing Guide
This guide describes how to make a change in the repository, and contribute this 
change back to the lab.

## Sanity with Git
This guide will show you how to make real, actual changes with Git. For many new
members, that can be scary, and you may feel as though you might type in a wrong
command and break everything! The good news is that this can't happen.

Git works by giving you your own personalized copy of our source code. This copy
can only be found on **your** computer! No one else has access to it. Therefore,
you can't really hurt anyone else's progress. Even if you run some commands that
mess up your copy, you can just ask for a new one! How sweet is that?

Moreso, we use branch protection, so you can't push code onto the robot without
approval from a leader first. Therefore, don't feel pressured to always type the right
command - you might screw up, and it's all okay!

Now, back to the guide...

## Find a project
If you don't yet know what you want to work on, see our [issues list](https://github.com/uf-mil/mil/issues).
We have a variety of projects to work on, and you can select a task in whatever
project you'd like!

## General contributing process
For those unfamiliar with git, this will the general idea.
1. Create a new branch off of the master branch (or branch off of another branch 
you want to fix).
1. Make changes on that new branch.
1. Add files that you will want to commit (You can think of a commit like a 
save file that you can go back to at any moment in time).
1. Create a commit (This is actually creating the "save file").
1. Push the commit to the remote branch.
1. Create a pull request on github that compares your branch to master branch.

Check out [this cool link](https://rogerdudler.github.io/git-guide/) if you want
another overview of Git!

## Create a new branch
Create a new branch for your changes `git checkout -b <branchname>`. Be sure to 
pick a descriptive name (usually 2-4 words, joined by dashes will do). The specific 
name of the branch generally does not matter.

## Viewing the status of files
You can see all the changes that have been made with these commands:

    $ git status # see the status of all files (what's changed, deleted, added, etc.)
    $ git diff # see the changes you've made

You may notice something called the "stage" and that files are either "staged" or
"unstaged". The stage covers everything that will be committed when you commit.
Usually, staged files are "ready" to be shown to someone else. Staged files shouldn't
usually contain sensitive information, files unrelated to MIL, etc.

It's sort of like if you have a folder with homework that's ready to turn in. You
may finish your homework two days before it's due and put it in this folder, "staging"
it. It's not officially turned in (that will happen when we commit), but it's ready to go!

## Adding file to be staged for a commit 
You can add all these changes to be staged like this:

    $ git add .

Or if you have only certain files you want to add:

    $ git add <file> <file...>

Be sure to only change the files you intend to and don't add any files that aren't needed.

## Committing changes
After a while, you'll want to make a **commit**. A commit is a point in time in the repository.
It holds the status of every file in the repository at a particular time. This is
super helpful - let's say you acidentally deleted an important file. You can go
back to the previous commit, and grab the file again.

You will need to provide a short description of your change when you commit it.

    $ git commit -m "Description of what changed goes here"

## Pushing your commit
Now, you have made one or more commits, but unfortunately, these changes are still
only available on your local copy. Let's change that, with pushing!

Pushing commits allows everyone in MIL to see your work. This allows leaders to 
approve your work and other MILers to build off of what you've built!

:::{danger}
Note that pushing your changes will make them available to everyone on the Internet.
Therefore, never commit sensitive info containing personal info about you or someone
else.
:::

Push your development branch to Github using:

    $ git push origin <branchname>

This `push`es your changes to the `origin` (aka GitHub) relative to the branch
`branchname` (the branch you made earlier).

You may need to use the `-u` flag on your first `git push`. This allows git to register
the branch you're pushing to as the one you always want to be pushing to.


## Creating a pull request
Now that your changes have been committed, it's time to submit them to be 
reviewed and then merged into the repository on Github.

To create a new pull request, head over to the `Pull Requests` tab of our GitHub
page and create a new pull request. Choose which branch to merge into `master` (or
whatever other branch you're trying to merge your code into), and a dialog should appear.
Add a title and body explaining what is changing. If you know what project your task
is related to, you can also connect your PR to a project on the right-hand side of
the interface.

## Request and wait for review
To ensure our code quality and robot safety, we require all code to be reviewed 
by at least one core developer before merging.

When you first opened your PR, you may have noticed a spinning yellow circle on
the page. This is our continuous integration. It's a friendly robot running your
code to check for any possible errors. If your code contains no errors, then a green
checkmark will appear, helping your reviewer to know that your changes are prime to accept.
If it doesn't pass, you'll see a bright red X, indicating that you need to review your
changes and fix something. See the next section below.

If you have a friend in MIL or someone with a good knowledge of the things you 
changed, you can assign them as a reviewer on GitHub. If it has been a few days 
and no one has looked at your pull request, you can also bug people in slack.
We strive for a 24-hour turn around for the first review.

## Amend pull request
Unless you are a true prodigy (or your reviewer is lazy), there will likely be 
some changes requested in your PR. After making the requested changes, you need 
to commit them and push them.

* You should merge in upstream changes before making your changes. See the section below.
* Look at the changes you made with `git status` and `git diff`.
* Add the files you wish to keep to staging `git add <file> <file...>` (or all files with `git add .`).
* Commit the changes with a descriptive message `git commit -m "<message>"`.
* Push your new commit to the same branch you submitted your PR from `git push origin <branchname>`.

Now it's time to wait for reviews again, this time hopefully you will be approved.

## Merging in upstream changes
If changes made by other developers are merged in before your pull request, you will have to update
you pull request to include these changes. This can sometimes be a tedious process if changes
were made to the same files you changed.

* Fetch the latest changes from upstream (this just caches them locally, not 
changing any of your code): `git fetch origin`.
* Backup your current branch in case anything goes wrong during the rebase with `git checkout -b <branchname>-backup`, 
and then go back to your original branch `git checkout <branchname>`.
* Rebase your changes to the latest master branch `git rebase -i origin/master`
* This will open up your text editor with a list of the commits that will be 
added on top of master. Be sure to comment out (by adding `#` before the line) 
any commits that are not yours.
* Save and close the text editor. Git will now attempt to rebase your changes 
on top of the master branch. If git detects a conflict, it will prompt you 
on how to manually fix them.
* Once you have gone through the whole process and git says the rebase was 
successful, push your updated branch `git push origin -f <branchname>`. Note 
the `-f` which you need as you have re-written history
