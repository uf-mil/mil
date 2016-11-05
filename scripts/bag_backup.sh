#!/bin/sh

# This script can be used to sync all new bags in the bags directory
# to an external hard drive. It should be set to run automatically when the
# system boots up by placing the following command in /etc/rc.local:
#
# # Backs up the bags to the external drive if it is available
# USER={user that owns the bags directory}
# INTERNAL_DIR={bags directory on the internal hard drive}
# EXTERNAL_DIR={bags directory on the external hard drive}
# sudo -u $USER -i screen -dmS bag-backup bash -i -c \
# "~/mil_ws/src/Navigator/scripts/bag_backup.sh $INTERNAL_DIR $EXTERNAL_DIR"
#
# In order to have the bags be copied automatically, plug the external drive
# into the computer and boot it. A screen session named 'bag-backup' will be
# created to display the progress of the transfer. Once this session closes,
# the system can be powered down and the drive can be removed. The script can
# also be run manually so long as the running user is the owner of the bag
# files. It will simply exit if the drive is not mounted.


#======================#
# Script Configuration #
#======================#

# The directories to sync are set in the command to run the script
INTERNAL_DIR=$1
EXTERNAL_DIR=$2

#================#
# Backup Command #
#================#

if [ -d $EXTERNAL_DIR ]; then
	echo "Synchronizing bags to the external drive mounted at $EXTERNAL_DIR"

	# Transfers bags to the external drive with rsync
	rsync --archive --recursive --compress --times --verbose \
	--human-readable --perms --chmod=u+rw,g+rw,o+r \
	--progress $INTERNAL_DIR/* $EXTERNAL_DIR/
else
	echo "Bag backup has been aborted because the external device is not mounted"
fi
