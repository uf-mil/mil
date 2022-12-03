#! /bin/bash

echo Enter keyword to delete
read -r word </dev/tty
pgrep -f "$word" >ptkword.txt

echo Kill all processes that contain: "$word" ? [Y/N]
read -r killall </dev/tty

while read -r p; do
	case "$killall" in
	y | Y) kill -9 "$p" ;;
	n | N) ;;
	esac
done <ptkword.txt
sleep 1

pgrep -f "$word" >ptkword.txt

while read -r p; do

	pname=$(ps -p "$p" -o comm=)
	echo Delete "$pname" ? [Y/N]
	read -r answer </dev/tty

	case "$answer" in
	y | Y) kill -9 "$p" ;;
	n | N) echo "NO" ;;
	esac
done <ptkword.txt
rm ptkword.txt

echo "Clean up finished :)"
