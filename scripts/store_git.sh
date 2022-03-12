#! /bin/sh
# Author: Cameron Brown

# From https://stackoverflow.com/a/16844327 - thanks!
RCol='\e[0m'    # Text Reset

# Regular           Bold                Underline           High Intensity      BoldHigh Intens     Background          High Intensity Backgrounds
Bla='\e[0;30m';     BBla='\e[1;30m';    UBla='\e[4;30m';    IBla='\e[0;90m';    BIBla='\e[1;90m';   On_Bla='\e[40m';    On_IBla='\e[0;100m';
Red='\e[0;31m';     BRed='\e[1;31m';    URed='\e[4;31m';    IRed='\e[0;91m';    BIRed='\e[1;91m';   On_Red='\e[41m';    On_IRed='\e[0;101m';
Gre='\e[0;32m';     BGre='\e[1;32m';    UGre='\e[4;32m';    IGre='\e[0;92m';    BIGre='\e[1;92m';   On_Gre='\e[42m';    On_IGre='\e[0;102m';
Yel='\e[0;33m';     BYel='\e[1;33m';    UYel='\e[4;33m';    IYel='\e[0;93m';    BIYel='\e[1;93m';   On_Yel='\e[43m';    On_IYel='\e[0;103m';
Blu='\e[0;34m';     BBlu='\e[1;34m';    UBlu='\e[4;34m';    IBlu='\e[0;94m';    BIBlu='\e[1;94m';   On_Blu='\e[44m';    On_IBlu='\e[0;104m';
Pur='\e[0;35m';     BPur='\e[1;35m';    UPur='\e[4;35m';    IPur='\e[0;95m';    BIPur='\e[1;95m';   On_Pur='\e[45m';    On_IPur='\e[0;105m';
Cya='\e[0;36m';     BCya='\e[1;36m';    UCya='\e[4;36m';    ICya='\e[0;96m';    BICya='\e[1;96m';   On_Cya='\e[46m';    On_ICya='\e[0;106m';
Whi='\e[0;37m';     BWhi='\e[1;37m';    UWhi='\e[4;37m';    IWhi='\e[0;97m';    BIWhi='\e[1;97m';   On_Whi='\e[47m';    On_IWhi='\e[0;107m';

color() {
    printf "$1"
}

# header display functions
hash_header() { echo "########################################"; }

clear
cat << EOF
$(color $Pur)
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
$(color $Yel)
$(hash_header)
Hola! Let's *git* your Git setup ready to go. You will need to do this at one point or another,
but doing it all in one place will help to ensure that your computer stores your credentials
quickly and securely.

This script will help you:
1. Store your Git name/email if not already set
2. Download libcrypt, a secure library for storing Git credentials
3. Help you create a personal access token for Git authentication
EOF
