# Installing Drivers for RTX 2080

Here is a quick guide for how to install the latest NVIDIA GPUs for the shuttle computer (which uses the NVIDIA GEFORCE GTX 970).

## Installing the NVIDIA Driver
First of all, you want to add the `ppa:graphics-drivers/ppa` repository into your computer system.

Run the commands:

    $ sudo apt-add-repository ppa:graphics-drivers/ppa
    $ sudo apt update

Update the packages that you downloaded with:

    $ sudo apt-get update
    $ sudo apt-get upgrade

Next, verify that your system recognizes the correct graphic card model and recommended driver by running the command:

    $ ubuntu-drivers devices

All of the recommended drivers for your computer should appear in the terminal like this:

    $ ubuntu-drivers devices
    == /sys/devices/pci0000:00/0000:00:01.0/0000:01:00.0 ==
    modalias : pci:v000010DEd000013C2sv00003842sd00002976bc03sc00i00
    vendor   : NVIDIA Corporation
    model    : GM204 [GeForce GTX 970]
    driver   : nvidia-driver-430 - third-party free
    driver   : nvidia-driver-390 - third-party free
    driver   : nvidia-driver-435 - distro non-free
    driver   : nvidia-driver-440 - third-party free recommended
    driver   : nvidia-driver-410 - third-party free
    driver   : nvidia-driver-415 - third-party free
    driver   : xserver-xorg-video-nouveau - distro free builtin`

To install the recommended driver automatically, run:

    $ sudo ubuntu-drivers autoinstall

To select a specific driver that needs to be installed, run the command:

    $ sudo apt install nvidia-<insert driver version here>

Once this is done installing, reboot the system with:

    $ sudo reboot

## Verifying that the correct GPU is being used and is in Default Compute Mode

The drivers that we install on this computer must be defaulted to Compute Mode. To verify that the correct driver is being used, run the command:

    $ nvidia-smi

An interface containing information about your NVIDIA driver should show up in your terminal.

On the top of the table that appears should be the Driver Version. Verify that this is the correct driver you want installed. Below this, under the GPU section, there will be a '0' or a '1'. If the GPU is defaulted to '0', then the driver is already in Compute Mode. If not (or if you want further verification that the driver is in compute mode), run:

    $ nvidia-smi -i 0 -c 0

If the device was already in Default Compute mode, a message will appear in the terminal.

## References
For references on how the driver was installed, visit these websites:

<https://linuxconfig.org/how-to-install-the-nvidia-drivers-on-ubuntu-18-04-bionic-beaver-linux>

<https://www.cyberciti.biz/faq/ubuntu-linux-install-nvidia-driver-latest-proprietary-driver/>

For how to switch the GPUs into Compute Mode, visit this website:

<https://stackoverflow.com/questions/31731535/switch-cuda-compute-mode-to-default-mode>
