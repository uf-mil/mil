We're currently running Ubuntu Server 14.04 w/ kernel version 4.2.0. As such working on the sub requires a one to SSH remotely into it. By passing the -X/Y arg when ssh'ing the following GUI utilities can be launched:

### XFE File Manager
If you need a quick way to do more complicated file actions XFE is provided. To access it, simply run: 

     xfe &

### Coriander 
Utility used for managing onboard cameras. It's incredibly robust, and exposes any functionality that is provided by libdc1394. To access it, simply run:
 
     coriander &
