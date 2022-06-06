# (Optional) Install proprietary software
MIL uses a few proprietary packages which cannot be distributed with the repo.
You are able to compile the repo and run simulations without them, but
they must be installed on the actual robots.

* `./scripts/install_bvtsdk` - installs the BlueView Imaging Sonar SDK
* `./scripts/install_flycap`- installs the Pointgrey Flycapture SDK

These scripts will prompt you for an encryption password. Ask a MIL leader for this.

# (Optional) Install UDEV rules
If you plan on running certain MIL sensors from your local machine
or are setting up the install on a robot, you must give your user
access to certain USB devices through UDEV.

`./scripts/install_udev_rules`
