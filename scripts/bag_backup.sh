#!/bin/sh

# This script can be used to sync all new bags in the local bags directory
# to the MIL fileserver. The autofs package can be installed and configured
# to automatically mount the fileserver when the system boots if it is
# reachable over the network. This script can also be set to run automatically
# on boot by placing the following lines in /etc/rc.local:
#
# # Backup the bags to the MIL fileserver if it is available
# USER={user that owns the bags directory}
# CATKIN_DIR={The directory of the user's catkin workspace}
# BAG_BACKUP_SCRIPT=$CATKIN_DIR/src/mil_common/scripts/bag_backup.sh
# LOCAL_BAGS_DIR={bags directory on the internal drive}
# REMOTE_BAGS_DIR={bags directory on the mounted MIL fileserver share}
# sudo -u $USER -i screen -dmS bag-backup bash -i -c \
# "$BAG_BACKUP_SCRIPT $LOCAL_BAGS_DIR $REMOTE_BAGS_DIR"
#
# In order for the bags to be copied automatically, connect the vehicle to
# the MIL network and boot it. A screen session named 'bag-backup' will be
# created to display the progress of the transfer. Once this session closes,
# the system can be powered down and the vehicle can be disconnected. The
# script can also be run manually so long as the running user is the owner of
# the bags directory. It will simply exit if the fileserver share is not
# mounted.


# The directories to sync are set in the command to run the script
LOCAL_BAGS_DIR=$1
REMOTE_BAGS_DIR=$2


if [[ -d $REMOTE_BAGS_DIR ]]; then
	echo "Synchronizing bags to the MIL fileserver share mounted at $REMOTE_BAGS_DIR"

	# Transfers bags to the fileserver with rsync
	rsync --archive --recursive --compress --times --progress\
	--human-readable --verbose $LOCAL_BAGS_DIR/* $REMOTE_BAGS_DIR/
else
	echo "Bag backup has been aborted because the MIL fileserver share is not mounted"
fi
