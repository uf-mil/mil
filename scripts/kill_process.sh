
# !/bin/bash


echo Enter keyword to delete
read word < /dev/tty
pgrep -f "$word" > ptkword.txt

echo Kill all processes that contain: "$word" ? [Y/N]
read killall < /dev/tty


while read p; do
        case "$killall" in
         y|Y ) kill -9 $p;;
         n|N ) ;;
        esac
done<ptkword.txt
sleep 1

pgrep -f "$word" > ptkword.txt

while read p; do

	pname=$(ps -p $p -o comm=)
        echo Delete $pname ? [Y/N]
        read answer < /dev/tty

        case "$answer" in
         y|Y ) kill -9 $p;;
         n|N ) echo "NO";;
        esac
done<ptkword.txt
rm ptkword.txt

echo "Clean up finished :)"
