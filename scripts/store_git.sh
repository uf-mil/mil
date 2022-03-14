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

read -r -p "$(color $Gre)Ready to start? [y/N] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
        ;;
    *)
        exit 1
        ;;
esac

clear
GitName="$(git config --global user.name)"
GitEmail="$(git config --global user.email)"
GitFixNeeded=0

if [ "$GitName" != "" ]
then
cat << EOF
$(hash_header)
$(color $Yel)You appear to have your Git name/email setup. Are the following
values correct? The name should be your first name, followed by your last name. 
The email shown below should be linked to your GitHub account.

    $(color $Pur)Name: ${GitName}
    Email: ${GitEmail}

EOF
read -r -p "$(color $Gre)Are the values correct? [y/N] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
        ;;
    *)
        clear
        echo $(hash_header)
        GitFixNeeded=1
        ;;
esac
else
cat << EOF
$(hash_header)
$(color $Yel)You do not have your Git name/email set up. These values are used
to link your commits to you and your GitHub account.

EOF
GitFixNeeded=1
fi

if [ $GitFixNeeded = 1 ]
then
cat << EOF
$(color $Yel)Enter your Git credentials below.

EOF

Done=1
while [ $Done = 1 ]
do
    read -r -p "$(color $Gre)Enter your name (First Last): " GitNameResponse
    read -r -p "$(color $Gre)Enter your email: " GitEmailResponse

cat << EOF
$(color $Yel)Please verify that the values below are correct for your name
and email.

    Name: ${GitNameResponse}
    Email: ${GitEmailResponse}

EOF
read -r -p "$(color $Gre)Are the values correct? [y/N] " GitNameVerifyResponse
case "$GitNameVerifyResponse" in
    [yY][eE][sS]|[yY]) 
        clear
        echo $(hash_header)
        echo $(color $Gre)The values have been saved. Proceeding to the next step in 5 seconds.
        sleep 5;
        Done=0
        ;;
    *)
        ;;
esac
done

fi

#####################
# gh setup
#####################

if ! type "gh" > /dev/null
then
clear
cat << EOF
$(hash_header)
$(color $Yel)We will now setup your device authentication with GitHub using the GitHub CLI.
You do not appear to have the CLI installed.
EOF

read -r -p "$(color $Gre)May we install it? [y/N] " response
case "$response" in
    [yY][eE][sS]|[yY]) 
        ;;
    *)
        exit 1
        ;;
esac

curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list > /dev/null
sudo apt update
sudo apt install gh

GhInstalledAgain=$(gh --version)
if [ $GhInstalledAgain != 0 ]
then

cat << EOF
$(hash_header)
$(color $Red)There was an issue with the installation. The GitHub CLI could not be installed.
Please report this as an error.
EOF
exit 1;

else

clear
cat << EOF
$(hash_header)
$(color $Yel)The GitHub CLI was successfully installed. We are now going to
authenticate you using the CLI.
EOF

fi
fi

clear
cat << EOF
$(hash_header)
$(color $Yel)You will now generate a personal access token (PAT).

Follow the steps below:
1. Sign into https://github.com.
2. In the top right corner, click on your profile, and then Settings.
3. On the left sidebar, click Developer Settings.
4. On the left sidebar again, click on Personal Access Tokens.
5. Near the top, click Generate Token and enter your password again.
6. Set the Note to explain what the token is being used for (such as your VM)
   and set the expiration time to whatever you would like. Whenever the token
   expires, you will need to run this script again and re-authenticate.
7. Select all checkboxes under the Permissions section of your new token.
8. Click Generate Token and copy the new token.
   $(color $Red)Do not refresh the page or the token will disappear!
EOF

read -r -p "$(color $Gre)Enter the PAT (it should start with ghp_): " response
echo "$response" > token.txt
gh auth login --with-token < token.txt
rm token.txt

clear
cat << EOF
$(hash_header)
$(color $Yel)You should now be authenticated with Git and GitHub!
EOF
