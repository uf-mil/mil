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

# Check user's software version
SOFTWARE=$(uname -s)

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
fi

# Check user's version of Ubuntu
UBUNTU_VERSION=$(lsb_release -r | awk '{print $2}')

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
fi

# Install git
clear
echo "Checking to see if ${tty_bold}git${tty_reset} is installed..."
git --version 2>&1 >/dev/null
GIT_IS_AVAILABLE=$?

if [ $GIT_IS_AVAILABLE -ne 0 ]
then
    echo "${tty_bold}${tty_blue}Attempting to install git... (This may require your password, which is expected.)"
    sudo apt install git
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
    fi
else
    echo "${tty_bold}${tty_green}Great! git is already installed."
fi
