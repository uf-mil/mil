# Extension Support
Sometimes, you may need to work on a project which requires custom drivers or SDKs
be installed on your computer. Below are some of these cases, and the install scripts
which can help to set up the extensions on your computer.

## Proprietary Software
MIL uses a few proprietary packages which cannot be distributed with the repo.
You are able to compile the repo and run simulations without them, but
they must be installed on the actual robots.

**Install the BlueView Imaging Sonar SDK:**

    $ ./scripts/install_bvtsdk # installs the BlueView Imaging Sonar SDK

**Install the Pointgrey Flycapture SDK:**

    $ ./scripts/install_flycap # installs the Pointgrey Flycapture SDK

These scripts will prompt you for an encryption password. Ask a MIL leader for this.

## UDEV Rules
If you plan on running certain MIL sensors from your local machine
or are setting up the install on a robot, you must give your user
access to certain USB devices through UDEV.

    $ ./scripts/install_udev_rules
