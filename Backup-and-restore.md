## Restoring
### Materials
To restore an image backup of NaviGator's hardrive you will need
1. A Live USB with Ubuntu 16.04
2. The hard drive you will format and restore NaviGator's image onto
3. A computer
4. A second hard drive/usb drive/network volume with the image backup files
### Instructions
1. Plug hard drives and USB drives into the computer
1. Reboot into the Ubuntu Live USB
1. Partition (in gparted) the hard drive with one btrfs partition for the whole file system, and boot partitions if needed (create a small 2MB partition at the top of the hard drive if the partition table is GPT, or a EFI partition if running on an EFI system)
1. Mount the new btrfs filesystem to a new directory ```mkdir ~/navigator && mount /dev/sdXY```
1. Change into the mounted filesystem ```cd ~/navigator```
1. Create the btrfs subvolumes needed for /home and /drew_bagnell ```sudo btrfs subvolume create home``` ```sudo btrfs subvolume create media/drew_bagnell```
1. Restore the tar backup for root, home, and drew_bagnell ```sudo tar -xvpzf /media/<backup hardrive>/<latest root backup file>.tar.gz -C ~/navigator --numeric-owner``` ```sudo tar -xvpzf /media/<backup hardrive>/<latest home backup file>.tar.gz -C ~/navigator/home --numeric-owner``` ```sudo tar -xvpzf /media/<backup hardrive>/<latest bag backup>.tar.gz -C ~/navigator/media/drew_bagnell --numeric-owner```
1. Install grub ```sudo grub-install /dev/sdX``` (note pass in the hard drive device, not the partition)
1. Reboot without Live USB plugged in, and you should boot into the NaviGator ubuntu install

## Common issues
Here are some solutions to issues encountered while restoring from a backup:
* If you are restoring on a new hard drive, you need to change /etc/fstab as mentioned below
* If you are restoring onto a new computer (motherboard), you will need to edit /etc/network/interfaces to use the new interface ids (like eth0, enp0s31f6)
* The Bootloader/EFI situation can cause a lot of problems. Here are some things I learned restoring onto various sytems:
  * If the hard drive is setup GPT and you want to use legacy BIOS boot, you need a small 2MB partition with the flag bios-grub at the start of the hard drive before you run grub-install
  * Legacy bios with a MRB partition system shouldn't require any additional partitions
  * grub-install may file from the LiveUSB, in which case you need to chroot to the mounted navigator install, see [this guide](http://logan.tw/posts/2015/05/17/grub-install-and-btrfs-root-file-system/)
  * For grub to see the file system you may also need to run ```update-grub``` (this may also require you to be in a chroot)
  * For an EFI system (either MRB/GPT), follow [this guide](https://help.ubuntu.com/community/UEFI). You need to create an EFI partition (300 MB or so, FAT32 filesystem, boot flag) and have it mount to /boot/efi
# Restoring with a new hard drive
In the event that NaviGator's hard drive changed, one more step is needed after the restore process to boot the system. 
1. Boot to an Ubuntu Live USB
1. Find the UUID of the system partition where the navigator image is installed ```blkid /dev/sda2```
1. Mount the partition and edit /etc/fstab, changing the UUID for the root partition (and subvolumes) to the new UUID