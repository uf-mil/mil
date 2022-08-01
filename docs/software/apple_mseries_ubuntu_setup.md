# Installing Ubuntu 18.04 on a Mac with an M-series processor (M1)

Installing Ubuntu 18.04 on an M1 Mac can be challenging, as the device does not permit dual booting another operating system. Therefore, the recommended way to install Ubuntu 18.04 is to use a virtual machine.

1. Download Ubuntu 18.04 Server for ARMv8. You can find this [here](https://cdimage.ubuntu.com/releases/18.04/release/).

2. Install Parallels Desktop. You can use another VM software that can load an `.iso` file if you would like, but Parallels has been tested and is confirmed to work. If you use Parallels, you will need to make a Parallels account to use the software.

3. Load the `.iso` file you downloaded earlier. You will see multiple localization prompts about which language to use, which keyboard layout to use, etc. Select US/English for the first two prompts, and then insert a USB drive into your computer. The contents of the USB drive should not matter.

4. In the Parallels menu at the top of the window, allow Parallels access to the USB drive you inserted. Click the USB icon, and then click on the specific USB drive you inserted into your computer. It should now show a check mark next to the name.

5. Continue proceeding through the Ubuntu Server installation process. There is no GUI, all steps are completed through the textual user interface. When partitioning your disk, you can use guided partitioning, which should partition all of the space Parallels allotted for that VM.

6. The installation process will ask if you want to install any recommended software (such as software to make your computer into a DNS server). Do not select any options, press `<Enter>`.

7. The installation process should be complete. You should see a command line interface. Run `sudo apt-get update && sudo apt-get upgrade` to get the latest list of packages.

8. Run `sudo tasksel`. Then, find `Ubuntu Desktop` and press `<Space>`. Then, press `<Enter>`. Then, run `sudo reboot now`.

9. You should now see a GUI interface. To sign in, use the username and password you setup before.
