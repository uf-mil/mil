#!/usr/bin/env bash

#Setting for the editor to use with tmuxinator edit <config>
export EDITOR=$(which vim)
alias tx=tmuxinator

# add completion for the tmuxinator alias
# https://raw.githubusercontent.com/tmuxinator/tmuxinator/master/completion/tmuxinator.bash
_tx() {
    COMPREPLY=()
    local word
    word="${COMP_WORDS[COMP_CWORD]}"

    if [ "$COMP_CWORD" -eq 1 ]; then
        local commands="$(compgen -W "$(tmuxinator commands)" -- "$word")"
        local projects="$(compgen -W "$(tmuxinator completions start)" -- "$word")"

        COMPREPLY=( $commands $projects )
    elif [ "$COMP_CWORD" -eq 2 ]; then
        local words
        words=("${COMP_WORDS[@]}")
        unset words[0]
        unset words[$COMP_CWORD]
        local completions
        completions=$(tmuxinator completions "${words[@]}")
        COMPREPLY=( $(compgen -W "$completions" -- "$word") )
    fi
}
# https://unix.stackexchange.com/a/445052
complete -F _tx tx

# link the tmuxinator configs in the MIL repo to the system tmuxinator config
# so they can be accessed without using the -p option
for f in $MIL_REPO/.tmuxinatorConfigs/*
do
	#ln "$f" ~/.config/tmuxinator/
	ln "$f" ~/.config/tmuxinator/ &>/dev/null
done
