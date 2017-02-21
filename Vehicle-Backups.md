It is important to back up the operating system on a vehicle whenever any major change is made to it. Usually, there should not be any major changes after the initial installation; however, there are some cases that warrant it. In order to back up a vehicle, boot to a USB image of Ubuntu on it and mount the vehicle's share on the [[MIL Fileserver]]. The restoration process is somewhat similar and has the same initial requirements. When backing up, both the individual partition and the full image backups should be created. When restoring, one or the other should be used for restoration. Unless you know what you are doing, the default should be to restore the full image.

# Setup Steps

Replace the default `/etc/apt/sources.list` file with a more comprehensive one:

    # See http://help.ubuntu.com/community/UpgradeNotes for how to upgrade to
    # newer versions of the distribution.
    deb http://us.archive.ubuntu.com/ubuntu/ xenial main restricted
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial main restricted

    ## Major bug fix updates produced after the final release of the
    ## distribution.
    deb http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates main restricted

    ## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu
    ## team. Also, please note that software in universe WILL NOT receive any
    ## review or updates from the Ubuntu security team.
    deb http://us.archive.ubuntu.com/ubuntu/ xenial universe
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial universe
    deb http://us.archive.ubuntu.com/ubuntu/ xenial-updates universe
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates universe

    ## N.B. software from this repository is ENTIRELY UNSUPPORTED by the Ubuntu 
    ## team, and may not be under a free licence. Please satisfy yourself as to 
    ## your rights to use the software. Also, please note that software in 
    ## multiverse WILL NOT receive any review or updates from the Ubuntu
    ## security team.
    deb http://us.archive.ubuntu.com/ubuntu/ xenial multiverse
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial multiverse
    deb http://us.archive.ubuntu.com/ubuntu/ xenial-updates multiverse
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-updates multiverse

    ## N.B. software from this repository may not have been tested as
    ## extensively as that contained in the main release, although it includes
    ## newer versions of some applications which may provide useful features.
    ## Also, please note that software in backports WILL NOT receive any review
    ## or updates from the Ubuntu security team.
    deb http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse
    deb-src http://us.archive.ubuntu.com/ubuntu/ xenial-backports main restricted universe multiverse

    # Important security updates for Ubuntu packages
    deb http://security.ubuntu.com/ubuntu xenial-security main restricted
    deb-src http://security.ubuntu.com/ubuntu xenial-security main restricted
    deb http://security.ubuntu.com/ubuntu xenial-security universe
    deb-src http://security.ubuntu.com/ubuntu xenial-security universe
    deb http://security.ubuntu.com/ubuntu xenial-security multiverse
    deb-src http://security.ubuntu.com/ubuntu xenial-security multiverse

Install a few utilities:

    sudo apt-get install pv secure-delete

# Backing Up a Partition

Mount the partition to a temporary mount point:

    sudo mount /dev/{partition_id} /mnt

Zero free space to increase image compressibility:

    sfill -fllvz /mnt

Create an archive backup of the filesystem (preserves permissions and file metadata):

    tar -cpvzf /{fileserver_mount_directory}/Backups/{further_separation_if_needed}/{vehicle_hostname}_{partition_name}_{date (in mmddyyyy)}.tar.gz .

An example of the backup naming convention is `mil-sub-sub8_root_02202017.tar.gz`

# Backing Up an Image

This assumes you have backed up all partitions on the drive except the swap partition first so that the free space on each partition was zeroed.

Zero free space on the swap partition to increase image compressibility:

    sswap -fllvz /dev/{swap_partition_id}

Create a compressed image of the drive:

    dd if=/dev/{drive_id} | pv | gzip > /{fileserver_mount_directory}/Backups/{further_separation_if_needed}/{vehicle_hostname}_image_{date (in mmddyyyy)}.gz

# Restoring a Partition

Mount the partition to a temporary mount point:

    sudo mount /dev/{partition_id} /mnt

Remove all existing files in the directory (please be extremely careful...):

    rm -Rf /mnt/*

Enter the mounted partition:

cd /mnt

Restore an archive backup of the filesystem (preserves permissions and file metadata):

    tar -xpvzf /{fileserver_mount_directory}/Backups/{further_separation_if_needed}/{vehicle_hostname}_{partition_name}_{date (in mmddyyyy)}.tar.gz

# Restoring an Image

Restore a compressed image of the drive:

    gzip -dc /{fileserver_mount_directory}/Backups/{further_separation_if_needed}/{vehicle_hostname}_image_{date (in mmddyyyy)}.gz | pv | dd of=/dev/{drive_id}