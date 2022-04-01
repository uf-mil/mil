# Contributing Guide
This guide describes how to make a change in the repository, and contribute this 
change back to the lab.

## Join the uf-mil github organization
Our current workflow does not use forks, meaning in order to contribute you
need write access to the upstream repository. To get access, contact one of the
software leaders through Slack.

## Find a project
If you don't yet know what you want to work on, see our [issues list](https://github.com/uf-mil/mil/issues).
We have a variety of projects to work on, and you can select a task in whatever
project you'd like!

## General contributing process
For those unfamiliar with git, this will the general idea.
1. Create a new branch off of the master branch (or branch off of another branch you want to fix)
1. Make changes on that new branch.
1. Add files that you will want to commit (You can think of a commit like a save file that you can go back to at any moment in time).
1. Create a commit (This is actually creating the "save file")
1. Push the commit to the remote branch.
1. Create a pull request on github that compares your branch to master branch.

Check out [this cool link](https://rogerdudler.github.io/git-guide/) if you want
another overview of Git!

## Create a new branch
Create a new branch for your changes `git checkout -b <branchname>`. Be sure to 
pick a descriptive name and make sure there is not [already a branch with that name](https://github.com/uf-mil/mil/branches).
The specific name of the branch generally does not matter.

## Viewing the status of files
You can see all the changes that have been made with these commands:

    $ git status # see the status of all files
    $ git diff # see the changes you've made

This also allows you to see what branch you are currenlty on and what is already 
staged to be committed.

## Adding file to be staged for a commit 
You can add all these changes to be staged like this:

    $ git add .

Or if you have only certain files you want to add:

    $ git add <file> <file...>

Be sure to only change the files you intend to and don't add any files that aren't needed.

## Committing changes
Now that you have some super awesome, well tested, well documented, and clean 
changes, it's time to submit them for review!

You will need to provide a short description of your change when you commit it.

    $ git commit -m "Description of what changed goes here"

## Pushing your commit

Push your development branch to Github using:

    $ git push origin <branchname>

This `push`es your changes to the `origin` (aka GitHub) relative to the branch
`branchname` (the branch you made earlier).

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
by at least one core developer before merging. If you have a friend in MIL or 
someone with a good knowledge of the things you changed, you can assign them 
as a reviewer on Github. If it has been a few days and no one has looked at 
your pull request, you can also bug people in slack. We strive for a 24-hour
turn around for the first review.

## Amend pull request
Unless you are a true prodigy (or your reviewer is lazy), there will likely be some changes requested in your PR.
After making the requested changes, you need to commit them and push them.

* You should merge in upstream changes before making your changes. See [below](#merging-in-upstream-changes).
* Look at the changes you made with `git status` and `git diff`.
* Add the files you wish  to keep to staging `git add <file> <file...>` (or all files with `git add .`).
* Commit the changes with a descriptive message `git commit -m "<message>"`.
* Push your new commit to the same branch you submitted your PR from `git push origin <branchname>`.

Now it's time to wait for reviews again, this time hopefully you will be approved.

## Merging in upstream changes
If changes made by other developers are merged in before your pull request, you will have to update
you pull request to include these changes. This can sometimes be a tedious process if changes
were made to the same files you changed.

* Fetch the latest changes from upstream (this just caches them locally, not changing any of your code) `git fetch origin`
* Backup your current branch in case anything goes wrong during the rebase `git checkout -b <branchname>-backup` then go back to your original branch `git checkout <branchname>`
* Rebase your changes to the latest master branch `git rebase -i origin/master`
* This will open up your text editor with a list of the commits that will be added on top of master. Be sure to comment out (by adding `#` before the line) any commits that are not yours
* Save and close the text editor. Git will now attempt to rebase your changes on top of the master branch. If git detects a conflict, it will prompt you on how to manually fix them.
* Once you have gone through the whole process and git says the rebase was successful, push your updated branch `git push origin -f <branchname>`. Note the `-f` which you need as you have re-written history
