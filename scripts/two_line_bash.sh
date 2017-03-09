#!/bin/bash

# Two line bash is a script that adds separators between commands to make
# their output more readable. It also includes the current time and the
# selected branch of the git repository the user is in. While it is enabled
# by default, two line bash can be disabled by creating a .disable_tlb file in
# the user's home directory. This can be accomplished with the following
# command: touch ~/.disable_tlb


function parse_catkin_workspace {
	PS_WORKSPACE=""
	local WORKSPACE_DIR="`echo $ROS_PACKAGE_PATH | cut -d ':' -f1 | sed 's@/src@@'`"
	if [ -f $WORKSPACE_DIR/.catkin_workspace ]; then
		PS_WORKSPACE="(catkin `echo $WORKSPACE_DIR | rev | cut -d "/" -f1 | rev`) "
	fi
}

function parse_git_branch {
	PS_BRANCH=""
	if [ -d .svn ]; then
		PS_BRANCH="(svn r$(svn info|awk '/Revision/{print $2}')) "
		return
	elif [ -f _FOSSIL_ -o -f .fslckout ]; then
		PS_BRANCH="(fossil $(fossil status|awk '/tags/{print $2}')) "
		return
	fi
	ref=$(git symbolic-ref HEAD 2> /dev/null) || return
	PS_BRANCH="(git ${ref#refs/heads/}) "
}

function parse_tlb_info {
	parse_catkin_workspace
	parse_git_branch
}


if [ ! -f ~/.disable_tlb ]; then
	RESET="\[\033[0m\]"
	RED="\[\033[0;31m\]"
	GREEN="\[\033[01;32m\]"
	BLUE="\[\033[01;34m\]"
	YELLOW="\[\033[0;33m\]"

	PROMPT_COMMAND=parse_tlb_info
	PS_LINE=`printf -- "- %.0s" {1..200}`
	PS_FILL=${PS_LINE:0:$COLUMNS}
	PS_INFO="$GREEN\u@\h$RESET:$BLUE\w "
	PS_CATKIN="$YELLOW\$PS_WORKSPACE"
	PS_GIT="$YELLOW\$PS_BRANCH"
	PS_TIME="\[\033[\$((COLUMNS-10))G\] $RED[\t]"
	export PS1="\${PS_FILL}\[\033[0G\]${PS_INFO}${PS_CATKIN}${PS_GIT}${PS_TIME}\n${RESET}\$ "
fi
