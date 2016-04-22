**Navigation Vessel**

* Gumstix board used in NAV vessel, running Linux
* Custom board with IMU, designed by ???

Linux disk commands:
* List disks/partitions: sudo fdisk -l
* List mounted partitions/disk space: df -h
* Unmount partition: sudo umount /dev/sdcX
* SD CARD to IMAGE: sudo dd if=/dev/sdX of=~/nav.img bs=512
* IMAGE to SD CARD: sudo dd if=~/nav.img of=/dev/sdX bs=512