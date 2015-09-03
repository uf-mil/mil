Git
===


### I want to retroactively change the commit message for my last commit

    git commit --amend -m "New Commit Message"

### I want to pull without a merge
    git pull --rebase

### I want to pull from the main repository, back to my local copy of my own fork
    git remote add upstream github.com:uf-mil/Sub8.git
    git pull upstream master

### I want to undo a commit
    git reset HEAD~1

The number after the "~" determines how many commits to go back. --hard will undo the changes and the commits, --soft will just undo the commits, but the changes will remain.