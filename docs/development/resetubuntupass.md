Forgot your password (AGAIN)? Don't worry, we got you ;)
---
Follow the steps for reseting your Ubuntu Password:

1. Boot up your device / VM

2. Go into the boot menu by either
    A. Holding down "SHIFT"
    B. Pressing "ESC"

    This will get you into the GRUB menu
    
![image text](https://i.stack.imgur.com/MQv6f.png)

3. Navigate to the 2nd option from the top (Top most Recovery Boot Option) and then hit "ENTER"
    This will get you to the Recovery Menu
![image text](https://i.stack.imgur.com/RRKur.png)

4. Now, the root prompt shows up like: root@ubuntu:~#

5. Remount the filesystem using: ``` mount -o remount,rw / ```

6. Next, you need to know your user name. You can do this by using: ``` ls /home ```

7. To change the password, use: ``` passwd userName ```, replacing userName with your user name
![image text](https://cdn.sstatic.net/Sites/askubuntu/img/site-background-image.png?v=29bccd27864c)
