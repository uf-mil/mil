
# !/bin/bash


echo What keyword would you like to delete?
read word < /dev/tty
pgrep -f "$word" > ptkword.txt

echo Would you like to kill all processes? [Y/N]
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
        echo Do you want to delete $p ? [Y/N]
        read answer < /dev/tty
        echo you responded $answer

        case "$answer" in
         y|Y ) kill -9 $p;;
         n|N ) echo "NO";;
        esac
done<ptkword.txt
rm ptkword.txt

echo "Clean up finished :)"
