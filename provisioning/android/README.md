# Android Robotics

## Main Idea

Android smart phones are basically half robot already and can be obtained for relatively little.

They already have many sensors that robots use:

* Camera
* Accelerometer / Gyro (although they usually arn't robotics grade)
* GPS

They are also:
* already rugged-ized
* self powered
* on the second hand market (especially if you dont mind a cracked screen), are much cheaper than a comparable raspberry PI.

There may be potential to lower the cost and development time of making new smaller, cheaper, and more experimental robots for the Machine Intelligence Laboratory if we could learn to harness this resource.

Also, if we ever did want a robot with a human interface of some kind, a touch screen and the android app building utilities is a pretty good place to start.

## How To Use

### Materials
* micro SDCARD (32GB - 128GB recommended)
* Android phone (not your daily driver preferably) that has a working sdcard reader and can run android 5.0+ (will require root priviledges)
* Linux laptop / desktop with micro sdcard reader

## Procedure

* run the install script for the linux computer and follow its instructions

`./scripts/install`

* Root the phone if not already https://www.cnet.com/how-to/how-to-easily-root-an-android-device/

* reset the phone to factory default via settings (not strictly nessesary, but recommended)

* connect the phone to wifi

* install Magisk and TWRP https://www.xda-developers.com/how-to-install-magisk/

* download the F-Droid APK https://f-droid.org/en/packages/com.matoski.adbm/

* plugin the phone to the linux computer via the usb cable

* enable usb debugging on the phone

* make sure the phone is detected by adb

`adb devices`

* install the F-Droid APK

`adb install ~/Downloads/F-Droid.apk`

* through F-Droid install `Termux` and `Termux Boot`

* use magisk to give the adb shell root permissions

`adb shell su`

open the magisk app and press on the shield, then enable the root permissions to shell

* insert the micro sdcard into the phone.

* in settings, format the micro sdcard

* eject the sdcard and put it into the linux computer

* find the sdcard's mount point

for me on ubuntu it was `/media/$USER/android`

* creat the ubuntu image
`SDCARD_SIZE_GB=100 SDCARD=/media/$USER/android ./scripts/create_ubuntu_img`

(for a 128 GB sdcard)

**NOTE: `SDCARD_SIZE_GB` is in GiB NOT GB**

* Wait for that to finish (it takes a long time)

* eject the micro sdcard

* use ADB to find the mount point for your sdcard, it should be under `/storage`

`adb shell ls /storage`

insert the sdcard into the phone

`adb shell ls /storage`

see if there is an sdcard id there, mine was `/storage/C47E-B2F6`

* configure the ubnutu image on the phone

`SDCARD=C47E-B2F6 ./scripts/configure_ubuntu_chroot`

**NOTE use your sdcard id**

* answer its prompts!

* test to make sure the ubuntu image works

`adb shell`

`su`

`cd /storage/C47E-B2F6`

`cd ubuntu`

`sh enter_chroot`

* you should now be in a bash session that looks like this

```
ali:/storage/C47E-B2F6/ubuntu # sh enter_chroot 
Mounting ubuntu chroot...
Setting environment vars
spaceswan@localhost:~$ 
```
* exit the ubuntu session

`exit`

* exit the adb shell

`exit`

* reboot the device

* unlock the device

* use magisk to give the termux boot root privileges

* reboot the phone

* unlock the phone

* use adb to find the device's ip address

`adb shell ip a`

* note the wlan0 inet address

* you should now be able to ssh into the `spaceswan` user at the wlan0 imet address (make sure you are on the same network as the phone)

`ssh spaceswan@<wlan0 inet address>`

### Recommended

#### Allocate More Swap space

Often android devices do not have enough memory to compile large catkin projects locally and the device will hard crash.

To remedy this, I have added a script to allocate additional swap space under `/data/swapfile`

To allocate N GiB of extra swap, run `SWAP_SIZE=N ./scripts/allocate_extra_swap`

This repo has been validated on:

* Motorola G6 (codename: ali)

Please make a pull request if you find any errors with this documentation / scripting infrastructure :)
