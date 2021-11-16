#! /bin/sh

# string formatters (thank you Homebrew!)
if [[ -t 1 ]]
then
  tty_escape() { printf "\033[%sm" "$1"; }
else
  tty_escape() { :; }
fi
tty_mkbold() { tty_escape "1;$1"; }
tty_underline="$(tty_escape "4;39")"
tty_blue="$(tty_mkbold 34)"
tty_green="$(tty_mkbold 32)"
tty_red="$(tty_mkbold 31)"
tty_bold="$(tty_mkbold 39)"
tty_reset="$(tty_escape 0)"

# header display functions
hash_header() { echo "########################################"; }

clear
cat << EOF


         &@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@#
       @@@@@@*,...................................................,/@@@@@@
     ,@@@@                                                            ,@@@@
     (@@@(                      /////&@* (@%////*                      &@@@*
     (@@@(                 #         %@* (@(         &                 &@@@*
     (@@@(                @@@@       %@* (@(      ,@@@@                &@@@*
     (@@@(               &@@@@@@%    %@* (@(    @@@@@@@#               &@@@*
     (@@@(              %/  @@@@@@@. %@* (@( ,@@@@@@&  #,              &@@@*
     (@@@(             *&     ,@@@@@@@@* (@@@@@@@@      @.             &@@@*
     (@@@(             @         (@@@@@* (@@@@@,         @             &@@@*
     (@@@(            @             @@@* (@@#            ,@            &@@@*
     (@@@(           &               %@* (@(              /#           &@@@*
     (@@@(          %/               %@* (@(               %,          &@@@*
     (@@@(         *@                %@* (@(                @.         &@@@*
     (@@@(         @                 %@* (@(                 @         &@@@*
     (@@@(        @                  %@* (@(                 ,@        &@@@*
     (@@@(  .@@@@@@@@            &@@@@@* (@@@@@#            @@@@@@@@   &@@@*
     (@@@#  .%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   &@@@*
     .@@@@.                                                           (@@@@
       @@@@@@%////////////////////////////////////////////////////(&@@@@@@
         #@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@/


$(hash_header)
${tty_bold}${tty_blue}Welcome to the welcome setup of the Machine Intelligence Laboratory at the University of Florida.
${tty_reset}This script will help you to get various software components of development setup through one simple script.

This setup will help to explain things along the way. It's okay if you feel overwhelmed - you don't have to know all of the setup commands and systems currently, and you may learn many of them during your time at MIL!

If you have issues, please report them here: -- URL --

${tty_bold}${tty_green}Would you like to continue? [Y/n]${tty_reset}
EOF
hash_header

read Step1Confirmation
if [ "$Step1Confirmation" != "Y" ]
then
    exit 0
fi
clear

# Check user's software version
SOFTWARE=$(uname -s)
echo "Checking to see if ${tty_bold}Linux${tty_reset} is installed..."

if [ "$SOFTWARE" != "Linux" ]
then
    clear
    hash_header
    cat << EOF
${tty_bold}${tty_red}Uh oh! You do not have Linux installed.
${tty_reset}
It is recommended that you install Ubuntu 18.04 before proceeding with the rest of the installation. Keeping the operating system consistent across our members' devices helps us to reduce the chance that code will work on one machine and not another machine.

If you would like to proceed anyway, you can do so. ${tty_underline}${tty_red}If you do not know what you are doing, do not move forward.${tty_reset}

${tty_bold}${tty_red}Would you like to proceed? [Y/n]${tty_reset}
EOF
    hash_header
    read StepLinuxIssueConfirmation
    if [ "$StepLinuxIssueConfirmation" != "Y" ]
    then
        exit 1
    fi
    clear
else
    echo "${tty_bold}${tty_green}Great! You are using Linux."
fi

# Check user's version of Ubuntu
UBUNTU_VERSION=$(lsb_release -r | awk '{print $2}')
echo "Checking to see if ${tty_bold}Ubuntu 18.04${tty_reset} is installed..."

if [ "$UBUNTU_VERSION" != "18.04" ]
then
    clear
    hash_header
    cat << EOF
${tty_bold}${tty_red}Uh oh! Your version of Ubuntu is not Ubuntu 18.04.
${tty_reset}
It is recommended that you install Ubuntu 18.04 before proceeding with the rest of the installation. Keeping the operating system consistent across our members' devices helps us to reduce the chance that code will work on one machine and not another machine.

If you would like to proceed anyway, you can do so. ${tty_underline}${tty_red}If you do not know what you are doing, do not move forward.${tty_reset}

${tty_bold}${tty_red}Would you like to proceed? [Y/n]${tty_reset}
EOF
    hash_header
    read StepUbuntuIssueConfirmation
    if [ "$StepUbuntuIssueConfirmation" != "Y" ]
    then
        exit 1
    fi
    clear
else
    echo "${tty_bold}${tty_green}Great! You are using Ubuntu 18.04."
fi

# git info prompt
clear
hash_header
cat << EOF
${tty_bold}Installing git...
${tty_reset}
git is a version control tool that helps developers like you to manage versions of their software. It can be accessed in the terminal by writing 'git' followed by arguments.

As you grow in MIL, you can use git to add changes to the MIL codebase. Some helpful commands include:

${tty_blue}git add file1.txt file2.txt ...${tty_reset}
    Adds several files to tracking index.

${tty_blue}git commit -m "This is a commit message."${tty_reset}
    Commits several files. This marks a formal change in the repository - a change that others will be able to see.

${tty_blue}git push${tty_reset}
    This pushes the local commits to the remote repository, hosted on GitHub.

These few commands can allow you to make several changes to the codebase and share them with others!

${tty_bold}${tty_green}Would you like to proceed? [Y/n]${tty_reset}
EOF
hash_header
read Step2Confirmation
if [ "$Step2Confirmation" != "Y" ]
then
    exit 1
fi
clear

# Install git
echo "Checking to see if ${tty_bold}git${tty_reset} is installed..."
git --version 2>&1 >/dev/null
GIT_IS_AVAILABLE=$?

if [ $GIT_IS_AVAILABLE -ne 0 ]
then
    echo "${tty_bold}${tty_blue}Attempting to install git... (This may require your password, which is expected.)"
    sudo apt install git
    clear
    echo "Checking to see if ${tty_bold}git${tty_reset} is installed..."
    git --version 2>&1 >/dev/null # improvement by tripleee
    GIT_IS_AVAILABLE=$?

    if [ $GIT_IS_AVAILABLE -ne 0 ]
    then
        clear
        hash_header
        cat << EOF
${tty_bold}${tty_red}Uh oh! git was unable to be installed.
${tty_reset}
Git is a helpful software tool that allows you to communicate with the code repository on GitHub. Without it, you will be unable to make changes.

Please contact another software team member for assistance.
EOF
        hash_header
    else
        echo "${tty_bold}${tty_green}Great! git has been succesfully installed."
    fi
else
    echo "${tty_bold}${tty_green}Great! git is already installed."
fi
