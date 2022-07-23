#! /bin/bash

rosnode kill --all
pgrep -f "gazebo" > ptk.txt
pgrep -f  "rosmaster" >> ptk.txt

echo Kill all processes? [Y/N]
read -r killall < /dev/tty


while read -r p; do
	case "$killall" in
	 y|Y ) kill -9 "$p";;
	 n|N ) ;;
	esac
done<ptk.txt
sleep 1
pgrep -f "gazebo" > ptk.txt
pgrep -f "rosmaster" >> ptk.txt



while read -r p; do
	pname=$(ps -p "$p" -o comm=)
	echo Delete "$pname" ? [Y/N]
	read -r answer < /dev/tty

	case "$answer" in
	 y|Y ) kill -9 "$p";;
	 n|N ) echo "NO";;
	esac
done<ptk.txt
rm ptk.txt
echo "Clean up finished :)"
