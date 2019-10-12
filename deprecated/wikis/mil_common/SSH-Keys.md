If you don't have an SSH key, you are wasting valuable time every time you use SSH. SSH has many different plugins to authenticate users, the most common of which being the Linux PAM daemon. PAM requires the user to enter their system password for each SSH session. SSH key authentication is based on Public Key Infrastructure (PKI). The SSH server encrypts data with the user's certificate and the connection is only accepted if the user's client can decrypt that data with their key and send it back to the server. Not only does this eliminate the need to type a password every time you connect, but it is also much more secure as the keys we use can be huge compared to a password.

# Generating an SSH Key

Before generating a new key, you should make sure you don't already have one:

    ls ~/.ssh

This command should show your known_hosts file in all cases. If you already have id_rsa and id_rsa.pub files, then you already have a key and should either just use that one (i.e. skip the generation step) or delete it and generate a new one. If there is anything else in the folder (e.g. authorized_keys2) and you do not know what it is, it is a security risk and should be deleted immediately!

To generate a brand new, shiny SSH key:

    ssh-keygen -b 4096

When you are asked where to save the file, just use the default location (~/.ssh/id_rsa) or you may have problems later on. I highly recommend setting a password for the key! It only needs to be entered once on your machine to unlock the key, not for each SSH session like a user password. If you are **really** lazy, you can leave it blank and will never be prompted for a password.

# Copying the Cert to a Server

Each server that you want to use the key to authenticate with needs to have a copy of the accompanying certificate. This simple command will do all the heavy lifting for you:

    ssh-copy-id <user>@<host>

* `user` - The username you want to log in with (this must be run for each separate usermane even if they are on the same machine)
* `host` - The IP address or hostname of the machine on the network

This will ask you to enter the password for that user. Once this is done, you should be able to log in without typing any kind of password.