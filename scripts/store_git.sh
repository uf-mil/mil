#! /bin/bash
# Author: Cameron Brown

Red='\e[0;31m'
Gre='\e[0;32m'
Yel='\e[0;33m'
Pur='\e[0;35m'

color() {
	printf "%b" "$1"
}

# header display functions
hash_header() { echo "########################################"; }

# Display header
clear
cat <<EOF
$(color "$Pur")
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
$(color "$Yel")
$(hash_header)
Hola! Let's *git* your Git setup ready to go. You will need to do this at one point or another,
but doing it all in one place will help to ensure that your computer stores your credentials
quickly and securely.

This script will help you:
1. Store your Git name/email if not already set
2. Download gh, the CLI interface to GitHub
3. Help you create a personal access token for Git authentication

EOF

read -r -p "$(color "$Gre")Ready to start? [y/N] " response
case "$response" in
[yY][eE][sS] | [yY]) ;;

*)
	exit 1
	;;
esac

###################
# Git setup
###################

# Check whether git name and email are configured
clear
GitName="$(git config --global user.name)"
GitEmail="$(git config --global user.email)"
GitFixNeeded=0

# Configure both if needed
if [ "$GitName" != "" ]; then
	cat <<EOF
$(hash_header)
$(color "$Yel")You appear to have your Git name/email setup. Are the following
values correct? The name should be your first name, followed by your last name.
The email shown below should be linked to your GitHub account.

    $(color "$Pur")Name: ${GitName}
    Email: ${GitEmail}

EOF
	read -r -p "$(color "$Gre")Are the values correct? [y/N] " response
	case "$response" in
	[yY][eE][sS] | [yY]) ;;

	*)
		clear
		hash_header
		GitFixNeeded=1
		;;
	esac
else
	cat <<EOF
$(hash_header)
$(color "$Yel")You do not have your Git name/email set up. These values are used
to link your commits to you and your GitHub account.

EOF
	GitFixNeeded=1
fi

if [ $GitFixNeeded = 1 ]; then
	cat <<EOF
$(color "$Yel")Enter your Git credentials below.

EOF

	Done=1
	while [ $Done = 1 ]; do
		read -r -p "$(color "$Gre")Enter your name (First Last): " GitNameResponse
		read -r -p "$(color "$Gre")Enter your email: " GitEmailResponse

		cat <<EOF
$(color "$Yel")Please verify that the values below are correct for your name
and email.

    Name: ${GitNameResponse}
    Email: ${GitEmailResponse}

EOF
		read -r -p "$(color "$Gre")Are the values correct? [y/N] " GitNameVerifyResponse
		case "$GitNameVerifyResponse" in
		[yY][eE][sS] | [yY])
			clear
			hash_header
			echo "$(color "$Gre")The values have been saved. Proceeding to the next step in 5 seconds."
			sleep 5
			Done=0
			;;
		*) ;;

		esac
	done

fi

#####################
# gh setup
#####################

if ! type "gh" >/dev/null; then
	clear
	cat <<EOF
$(hash_header)
$(color "$Yel")We will now setup your device authentication with GitHub using the GitHub CLI.
You do not appear to have the CLI installed.
EOF

	read -r -p "$(color "$Gre")May we install it? [y/N] " response
	case "$response" in
	[yY][eE][sS] | [yY]) ;;

	*)
		exit 1
		;;
	esac

	curl -fsSL https://cli.github.com/packages/githubcli-archive-keyring.gpg | sudo dd of=/usr/share/keyrings/githubcli-archive-keyring.gpg
	echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/githubcli-archive-keyring.gpg] https://cli.github.com/packages stable main" | sudo tee /etc/apt/sources.list.d/github-cli.list >/dev/null
	sudo apt update
	sudo apt install gh

	GhInstalledAgain=$(gh --version)
	if [ "$GhInstalledAgain" != 0 ]; then

		cat <<EOF
$(hash_header)
$(color "$Red")There was an issue with the installation. The GitHub CLI could not be installed.
Please report this as an error.
EOF
		exit 1

	else

		clear
		cat <<EOF
$(hash_header)
$(color "$Yel")The GitHub CLI was successfully installed. We are now going to
authenticate you using the CLI.
EOF

	fi
fi

clear
cat <<EOF
$(hash_header)
$(color "$Yel")You will now generate a personal access token (PAT).

A web browser will open that allows you to create a new token. You should just
need to click Generate Token at the bottom of the dialog, after signing in.

EOF

read -r -p "$(color "$Gre")Open the browser? [y/N] " response
case "$response" in
[yY][eE][sS] | [yY])
	xdg-open "https://github.com/settings/tokens/new?scopes=repo,admin:public_key,gist,read:org&description=MIL+token"
	;;
*)
	exit 1
	;;
esac

read -r -p "$(color "$Gre")Enter the PAT (it should start with ghp_): " response
echo "$response" >token.txt
gh auth login --with-token <token.txt
rm token.txt

clear
cat <<EOF
$(hash_header)
$(color "$Yel")You should now be authenticated with Git and GitHub!
EOF
