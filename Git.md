If you want to _**understand**_ the git versioning model and not just memorize commands and when to run them, read [this](https://www.sbf5.com/~cduan/technical/git/).

### Forking Workflow

The MIL software team is using the git "Forking Workflow". The general idea is to fork one of our repositories, create a feature branch, make your changes, and then push your feature branch to your fork. GitHub will allow you to open a pull request to merge your feature branch into the repository's master branch. At this point, a maintainer will provide you feedback on your changes. You can add commits to your pull request simply by creating the new commit and pushing to the relevant branch of your fork. Once the pull request is accepted, the maintainer will merge your pull request into the master branch. For a more in-depth tutorial, see [this page](https://www.atlassian.com/git/tutorials/comparing-workflows/forking-workflow).

#### What do I do if I have multiple feature branches open and one gets merged into master? 

Once you have completed one feature branch and it is merged into master, you can pull the latest changes into your own local master branch (which should be tracking the upstream/master branch). You can merge your local master branch into your other feature branches. This could potentially cause merge conflicts and may result with an extra merge commit (which is OK and better than rebasing, which potentially rewrites history and makes things very sad). 

### I want to retroactively change the commit message for my last commit

    git commit --amend -m "New Commit Message"

### I want to pull without a merge
    git pull --rebase

### I want to pull from the main repository, back to my local copy of my own fork
    git pull upstream master

### I want to undo a commit
    git reset HEAD~1

The number after the "~" determines how many commits to go back. --hard will undo the changes and the commits, --soft will just undo the commits, but the changes will remain.