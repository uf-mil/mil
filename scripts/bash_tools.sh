#!/bin/bash
GOODCOLOR='\033[1;36m'
WARNCOLOR='\033[1;31m'
NOCOLOR='\033[0m'
GOODPREFIX="${GOODCOLOR}INSTALLER:"
WARNPREFIX="${WARNCOLOR}ERROR:"

# Sane installation defaults for no argument cases
CATKIN_DIR=~/repos/catkin_ws
REQUIRED_OS="trusty"
SEMAPHORE=false

instlog() {
    printf "$GOODPREFIX $@ $NOCOLOR\n"
}

instwarn() {
    printf "$WARNPREFIX $@ $NOCOLOR\n"
}

ros_git_get() {
    # Check if it already exists
    # ex: ros_git_get git@github.com:jpanikulam/ROS-Boat.git
    # Also checks https automatically!

    NEEDS_INSTALL=true;
    INSTALL_URL=$1;
    builtin cd $INSTALL_FOLDER
    for folder in "$INSTALL_FOLDER"/*; do
        if ! [ -d $folder ]; then
            continue;
        fi

        builtin cd $folder
        if ! [ -d .git ]; then
            # instlog "$folder not a git repository"
            continue;
        fi
        LOCAL_BRANCH=`git name-rev --name-only HEAD`
        TRACKING_BRANCH=`git config branch.$LOCAL_BRANCH.merge`
        TRACKING_REMOTE=`git config branch.$LOCAL_BRANCH.remote`
        REMOTE_URL=`git config remote.$TRACKING_REMOTE.url`
        if python -c "import re; _, have_url = re.split('https://github.com|git@github.com:', '$REMOTE_URL');_, want_url = re.split('https://github.com|git@github.com:', '$INSTALL_URL'); exit(have_url != want_url)"; then
            instlog "Already have package at url $INSTALL_URL"
            NEEDS_INSTALL=false;
            break;
        fi
        builtin cd $INSTALL_FOLDER
    done
    if $NEEDS_INSTALL; then
        instlog "Installing $INSTALL_URL in $INSTALL_FOLDER"
        git clone -q $INSTALL_URL --depth=1
    fi
}
