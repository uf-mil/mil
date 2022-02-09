# !/bin/bash

rosnode kill --all
pgrep -f "gazebo" > ptk.txt
pgrep -f  "rosmaster" >> ptk.txt

echo Kill all processes? [Y/N]
read killall < /dev/tty


while read p; do
	case "$killall" in
	 y|Y ) kill -9 $p;;
	 n|N ) ;;
	esac
done<ptk.txt
sleep 1
pgrep -f "gazebo" > ptk.txt
pgrep -f "rosmaster" >> ptk.txt



while read p; do
	pname=$(ps -p $p -o comm=)
	echo Delete $pname ? [Y/N]
	read answer < /dev/tty

	case "$answer" in
	 y|Y ) kill -9 $p;;
	 n|N ) echo "NO";;
	esac
done<ptk.txt
rm ptk.txt
echo "Clean up finished :)"

