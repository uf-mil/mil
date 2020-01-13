# File Server
MIL has a file server we use to store bag files, videos, etc.
This fileserver is accessable from anywhere on the UF network.

## Mounting (Ubuntu)
Run `./scripts/mount_fileserver <username> <share>`
The username will be the username of your MIL active directory account. Reach out in slack if you don't have one.
The share is which file share you want. You can chose one of the following:

* mil
* navigator
* subjugator

To unmount all shares, run `./scripts/umount_fileserver`
