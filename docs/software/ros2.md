# Migrating to ROS 2

It's time -- we're migrating to ROS 2, the next major version of ROS! Here, we'll document the process of setting up your computer to develop for ROS2, the migration process, and any other relevant information.

## System Requirements

Autonomous robotics is computationally expensive, especially when running simulations.
Your VM or computer should have at least 4GB, although it will run much faster with
8GB of RAM or higher. You should have at least a dual-core system, but using
more cores will allow compilation to run much faster.

If you are using a virtual machine, you can change the amount of RAM and the
number of CPU cores allotted to the virtual machine by visiting the settings
of your VM software.

## Setting Up Ubuntu

### Choosing an Install Method

You will need to install **Ubuntu 24.04 LTS** (disk image downloads linked below),
the main operating system supported by ROS and our tools. Our tools are not setup
to work on other operating systems, including other Linux distributions, macOS,
and Windows.

To use Ubuntu in MIL, you have two options:

1. Use a **virtual machine**. A virtual machine uses your current operating system
to virtually host another operating system (in this case, Ubuntu). This could
cause your Ubuntu environment to run slow (in some cases, it might be unusable)
and may cause you to experience issues with certain programs.

2. **Dual-boot your computer.** Dual booting will allocate a piece of your computer's
storage to a new operating system, which will be installed on your computer directly.
This dual-booted solution will not run on top of your current operating system, but
will instead run by directly using your computer's resources.

Dual-booting your computer is highly recommended over using a virtual machine,
as it allows your Ubuntu setup to tap directly into your computer's resources.
However, it's totally your choice.

:::{warning}
If you **have an M-series (ARM) Mac computer**, you will need to use a
virtual machine as these systems are not able to have two operating systems installed
at once. Intel-based Macs may also experience some issues with dual-booting Ubuntu.
:::

The following diagram should give you insight into which method is best for you.
Note that the diagram is only a recommendation. If you have installed Linux before,
or know of a technical issue with your computer, you may know that a different
method is best for you.

![Installation Guide](/software/installation.png)

### Installing Ubuntu

Here are the links to appropriate `.iso` (disk image) files. Note that you will
need to choose the correct disk image based on your computer's architecture. You
do not always need to download these files - they are only needed for some installation
methods.

| Architecture | URL |
| ------------ | --- |
| AMD64 (most Windows computers, Intel-based Mac computers) | [ubuntu-24.04.1-desktop-amd64.iso](https://releases.ubuntu.com/noble/ubuntu-24.04.1-desktop-amd64.iso) |
| ARM64 (Apple Silicon Mac computers) | 1. Ubuntu releases (faster): [noble-desktop-arm64.iso](https://cdimage.ubuntu.com/noble/daily-live/current/noble-desktop-arm64.iso)<br>2. MIL mirror (slower): [noble-desktop-arm64.iso](https://mil.ufl.edu/software/noble-desktop-arm64.iso) |

For specific installation methods, please choose one of the methods from the [getting started page](/software/getting_started), and substitute the Ubuntu 24.04 iso for the Ubuntu 20.04 iso file where needed.

### Cloning our repository

Unlike ROS 1, our repository will no longer live in `~/catkin_ws/src/`. Now, it's new home will be your home directory!

```bash
$ sudo apt update
$ sudo apt upgrade
$ sudo apt install git
$ git clone -j8 https://github.com/uf-mil/mil2
```

### Installing ROS 2

Our install script has been ported over to ROS 2, so it should be a simple installation:

```bash
$ cd mil2
$ ./scripts/install.sh
```

### Installing `pre-commit`

Don't forget to install `pre-commit` to check your changes before committing!

```bash
$ pip3 install -U pre-commit
$ pre-commit install
$ pre-commit run --all-files # should show no errors!
yamllint.................................................................Passed
black....................................................................Passed
... etc.
```

## Learning about ROS 2

While ROS 2 has many parallels to ROS 1, it will be worth your time to
review a little bit about the changes from ROS 1. These pages are good
reads before working with ROS 2:

* [ROS docs on "Migrating from ROS 1"](https://docs.ros.org/en/jazzy/How-To-Guides/Migrating-from-ROS1.html)
* [Using `colcon` to build packages](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)
* [Image: ROS 1 vs ROS 2 architecture](https://www.researchgate.net/profile/Takuya-Azumi/publication/309128426/figure/fig1/AS:416910068994049@1476410514667/ROS1-ROS2-architecture-for-DDS-approach-to-ROS-We-clarify-the-performance-of-the-data.png)
