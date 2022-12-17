# Accessing Zobelisk
In MIL, we have a high performance desktop computer called Zobelisk. We leverage
our development containers to run multiple robot solutions and simulations at
once all in separate isolated environments connected to the network.

:::{note}
In order to access Zobelisk, you must have an account on Zobelisk and be
connected to the MIL network.

In order to gain a login for Zobelisk, please reach out to a software lead.
:::

## Logging In
To access the basic bash shell associated with your user account, you can use standard
SSH protocol. This can be helpful if you'd like to store long-term material under
your username on Zobelisk.

To do so, sign in with `ssh` on your own Ubuntu installation:

    $ ssh <username>@192.168.37.176

The IP address is the IP address of Zobelisk on the MIL network.

## Get your UID
After signing in over `ssh`, you can get your UID (user ID) assigned to your username
by Linux:

    zobelisk$ echo $UID

This should print out a number around 1000. This number is your UID!

## Using `sshfs` with Docker
Great! Now that you've successfully signed into Zobelisk, let's take it to the next
level.

If you'd like, you can use `sshfs` with Docker to run programs safely on Zobelisk.
This means that instead of running programs in a bash terminal, that you will
instead run them in a containerized environment (a fresh install of Linux!).


:::{warning}
Currently, the MIL Software Team has reduced its usage of Docker compared to
previous editions of the team. Therefore, these steps are not required, and
we believe that you will be safe running most programs in a bash terminal
on Zobelisk.

The below scripts have not been thoroughly tested by this edition of the software
team.
:::

To use a helpful script for this, try running

    $ ./scripts/sshzobelisk <username> <UID>

This script does several things:

1. Sets up an `openssh` server between you and Zobelisk.
1. Generates an ssh key so that you can securely connect to Zobelisk.
1. Starts (and then detaches from) a new `tmux` session capable of running ROS (the
`$ROS_MASTER_URI` variable has been exported to it).
1. Generates and launches a new Docker container with your copy of the repository
(through the `ssh` server) and MIL-related tools.

After the repository finished building, you can jump into the `tmux` session and
began working on your project.

## Test Your Setup
In the remote container, launch SubjuGator with

    <username>@192.168.36.176$ roslaunch subjugator_launch gazebo.launch

Then in the tmux session running on your computer, run

    <username>@192.168.36.176$ rostopic echo /odom

If your setup is working correctly, you should see the changing odometry values of
a moving sub simulation.

## Log out

To log out of the remote container, run

    <username>@192.168.36.176$ exit
