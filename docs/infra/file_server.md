# File Server
MIL has a file server we use to store bag files, videos, etc.
This fileserver is accessable from anywhere on the UF network.

:::{warning}
These docs have not been updated in a considerable amount of time. Please use
caution when attempting to connect to the file server.
:::

## Mounting (Ubuntu)
To connect, use

    $ ./scripts/mount_fileserver <username> <share>

The username will be the username of your MIL active directory account. Reach out in slack if you don't have one.
The share is which file share you want. You can chose one of the following:

* mil
* navigator
* subjugator

To unmount all shares, run 

    $ ./scripts/umount_fileserver
