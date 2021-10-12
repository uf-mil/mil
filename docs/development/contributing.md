# Contributing Guide
This guide describes how to get a local copy of the repository, make a change, and 
contribute this change back to the lab.

Note this guide does not explore how to build or test changes to code (which should be done
prior to submitting your changes). For this information, view the [Development Guide](development_guide).

## Join the uf-mil github organization
Our current workflow does not use forks, meaning in order to contribute you
need write access to the upstream repository. See the [Onboarding Guide](/docs/onboarding)



## Install git
To contribute changes, you will need to have a git client installed. This program will be used to track and upload your changes.


**Ubuntu**

`sudo apt install git`

**Windows**

For using git in windows, we suggest you use the [Github Desktop Client](https://desktop.github.com/)

## Cloning the repository

You need to clone (download) a copy of the repository onto your computer so you can make changes.

**Ubuntu**

First clone the upstream (MIL's fork) version of the repo.
It is recommended that you first create a catkin workspace
and clone it into the `src` or that workspace.

`mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src`

Then clone the repo:

(the `--recurse-submodules` is essential, as that is what pulls the submodules, `-j8` is a speed optimization that downloads up to 8 submodules at a time)

`git clone --recurse-submodules -j8 https://github.com/uf-mil/mil.git`

**Windows**
Use the [Github Desktop GUI](https://help.github.com/en/desktop/contributing-to-projects/cloning-a-repository-from-github-to-github-desktop)

## Find a project
If you don't yet know what you want to work on, see our [issues list](https://github.com/uf-mil/mil/issues), the TODO column of the [SCUM board](https://github.com/orgs/uf-mil/projects/2), or ask around in slack.

## Make and test your changes!
Now its time to contribute! You may find these guides helpful in making and testing changes:

* [Development guide](/docs/development/development_guide)
* [Adding Documentation](/docs/development/adding_documentation)

Be sure to only change the files you intend to and don't add any files that aren't needed.


## Committing changes
Now that you have some super awesome, well tested, well documented, and clean changes, it's time to submit them for review!

You will need to provide a short description of your change when you commit it.


**Ubuntu**

* Create a new branch for your changes `git checkout -b <branchname>`. Be sure to pick a descriptive name and make sure there is not [already a branch with that name](https://github.com/uf-mil/mil/branches)
* Look at the changes you made with `git status` and `git diff`
* Add the files you wish  to keep to staging `git add <file> <file...>` (or all files with `git add .`
* Commit the changes with a descriptive message `git commit -m "<message>"`
* Push your development branch to Github `git push origin <branchname>`

**Windows**
Follow [Github's guide for the Windows client](
https://help.github.com/en/desktop/contributing-to-projects/committing-and-reviewing-changes-to-your-project)

## Creating a pull request
Now that your changes have been committed, it's time to submit them to be reviewed and then merged into the repository on Github.

Follow [Github's guide for creating pull requests](https://help.github.com/en/desktop/contributing-to-projects/creating-a-pull-request)


## Request and wait for review
To ensure our code quality and robot safety, we require all code to be reviewed by at least one developer before merging.
If you have a friend in MIL or someone with a good knowledge of the things you changed, you can assign them as a reviewer on Github.
If it has been a few days and no one has looked at your pull request, you can also bug people in slack. We strive for a 24hour
turn around for the first review.

## Amend pull request
Unless you are a true prodigy (or your reviewer is lazy), there will likely be some changes requested in your PR.
After making the requested changes, you need to commit them and push them.

**Ubuntu**
* You should merge in upstream changes before making your changes. See [below](#merging-in-upstream-changes)
* Look at the changes you made with `git status` and `git diff`
* Add the files you wish  to keep to staging `git add <file> <file...>` (or all files with `git add .`
* Commit the changes with a descriptive message `git commit -m "<message>"`
* Push your new commit to the same branch you submitted your PR from `git push origin <branchname>`

Now it's time to wait for reviews again, this time hopefully you will be approved.


## Merging in upstream changes
If changes made by other developers are merged in before your pull request, you will have to update
you pull request to include these changes. This can sometimes be a tedious process if changes
were made to the same files you changed.

**Ubuntu**
* Fetch the latest changes from upstream (this just caches them locally, not changing any of your code) `git fetch origin`
* Backup your current branch in case anything goes wrong during the rebase `git checkout -b <branchname>-backup` then go back to your original branch `git checkout <branchname>`
* Rebase your changes to the latest master branch `git rebase -i origin/master`
* This will open up your text editor with a list of the commits that will be added on top of master. Be sure to comment out (by adding `#` before the line) any commits that are not yours
* Save and close the text editor. Git will now attempt to rebase your changes on top of the master branch. If git detects a conflict, it will prompt you on how to manually fix them.
* Once you have gone through the whole process and git says the rebase was successful, push your updated branch `git push origin -f <branchname>`. Note the `-f` which you need as you have re-written history
