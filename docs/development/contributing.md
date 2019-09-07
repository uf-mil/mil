# Contributing Guide
To contribute changes to this repository (code, documentation, etc) use the following guide.

## Install git
> Note: if you already went through the install process for MIL software developement, this is already done

To contribute changes, you will need to have a git client installed. This program will be used to track and upload your changes.


**Ubuntu**

`sudo apt install git`

**Windows**

For using git in windows, we suggest you use the [Github Desktop Client](https://desktop.github.com/)

## Forking and cloning the repository
> Note: if you already went through the install process for MIL software developement, this is already done

You next need to create a fork of [MIL repository](https://github.com/uf-mil/mil) on your own github account (create one of if don't have one already). You also need to clone (download) a copy of the repository onto your computer

Follow [Github's documentation for forking](https://help.github.com/en/articles/fork-a-repo)


## Making changes and commiting
Make the changes you wish to contribute to your local clone of the repository above. Be sure to only change the files you intend to and don't add any files that aren't needed. You will need to provide a short description of your change when you commit it.

**Ubuntu**

* Create a new branch for your changes `git branch -b <name>`
* Look at the changes you made with `git status` and `git diff`
* Add the files you wish to keep to staging (or all files with `git add .`
* Commit the changes with a descriptive message `git commit -m "<message>"`

**Windows**
Follow [Github's guide for the Windows client](
https://help.github.com/en/desktop/contributing-to-projects/committing-and-reviewing-changes-to-your-project)

## Creating a pull request
Now that your changes have been commited, it's time to submit them to be reviewed and then merged into the repository on Github.

Follow [Github's guide for creating pull requests](https://help.github.com/en/desktop/contributing-to-projects/creating-a-pull-request)
