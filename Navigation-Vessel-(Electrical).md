**Navigation Vessel (Electrical)**

Overall info:
* Gumstix board used in NAV vessel, running Linux
* Custom board with IMU, designed by ???
* FPGA is on custom board, provides basic level shifting functionality - configuration image is provided through Gumstix
* Board is mounted to side with Ethernet bulkhead connector, very short cables from pressure sensor and DVL

Linux disk commands (Device: sdX, Partition: sdXX):
* List disks/partitions: sudo fdisk -l
* List mounted partitions/disk space: df -h
* Unmount partition: sudo umount /dev/sdXX
* SD CARD to IMAGE: sudo dd if=/dev/sdX of=~/nav.img bs=512
* IMAGE to SD CARD: sudo dd if=~/nav.img of=/dev/sdX bs=512