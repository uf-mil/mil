# Using The Development Server
In MIL, we have a high performance desktop computer called Zobelisk. We leverage our development containers to run multiple robot solutions and simulations at once all in separate isolated environments connected to the network.

*NOTE: in order to continue this tutorial, you must have an account on Zobelisk and be connected to the mil network. Ask a senior member for either of these.*

## What you need to know

### Username
If you do not know your username, ask a senior member to help you


### User id
If you do not know your user id,
ssh directly into zobelisk under your user

`ssh <username>@192.168.37.176`

and run

`echo $UID`

this will print out your user id.

ie: `1001`

Now logout of Zobelisk

`exit`


## Run The Script to Log You In
Now run

`./scripts/sshzobelisk <username> <user id>`

with all the values you just found.

*NOTE: If this is your first time, you will make an ssh key for your zobelsik account, follow the instructions on the screen.*

*NOTE: You may find yourself compiling the repository if this is your first time logging in. It will take a minute to finish.*

*NOTE: Gui programs are not going to work if you run them in the remote container.*

You are now logged into a remote container running on Zobelisk with its source code linked back to the source code on your computer.

So if you make a change to a file in the mil repo on your computer in whatever text editor you want, the changes will appear instantly in the docker container. This is to help facilitate people who like to use gui text editors.

Also, a new tmux session has been initiated on your computer with the correct environment variables for running ROS on the development server.

*NOTE: We currently do not have the gazebo gui working over network, so it will not work when running code on the development server.*

Now we will connect to the newly created tmux session. Open a new terminal window with

`Ctrl + Alt + t`

(if running default Ubuntu)

run 

`tmux ls`

look for the most recently created session and get its name (values all the way to the left).

then attach to the session with

`tmux a -t <name>`

In this tmux session, all the ros data will be avalible to you.

## Verify that everything is working

in the remote container, launch SubjuGator with

`roslaunch sub8_launch gazebo.launch`

Then in the tmux session running on your computer, run

`rostopic echo /odom`

And you should see a bunch of numbers flying up.

## Log out

To log out of the remote container, run

`exit` 

*NOTE: `exit` and `Ctrl + d` are equivilent.*

Then you will be in a normal bash session on Zobelisk. You should see a green 

`<username>@zobelisk:~$` 

(if using default Ubuntu terminal)

This is only for when you need to interact directly with your user on the server.

You can also just logout of this with

`exit`

Then you will need to retype in your sudo password on your computer to finish if you have been logged in for more than a few minutes.

Log out of the tmux session normally with 

`exit`

## How Does it Work

### Upon running the sshzobelisk script
#### The user's computer 

1. makes an ssh key for zobleisk if you do not already have one (so do not have to type in your zobelisk password everytime).

1. starts an ssh server.

1. exports the remote ros master uri

1. starts and detaches from a tmux session now with the correct ros mater uri

1. opens an ssh session on zobelisk.

#### On Zobelsik, the script automatically

1. uses  sshfs to link the repo on the user's computer to the repo under the user's account on zobelisk

1. builds the dev container (also compiles if needed)

1. launched the dev container on the host network

#### On exiting the remote conatiner

1. A bash session is opened under the user account.

#### After exiting the bash session.

1. The network link (sshfs) for the mil repo is unmounted.

1. The ssh server on the user computer is shutdown (requires sudo).




