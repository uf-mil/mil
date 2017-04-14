These are just a few notes on how pool days should be run.

Checklist:

* Charge batteries
* Update `~/mil_ws` to the current master branches for all repositories
* Seal both vessels
* Packing list
    * Tether
    * Gigabit ethernet switch
    * Ethernet cables for everyone
    * The testing gateway box and power cable
    * Telodyne imaging sonar cables and PoE injector
    * Power strips and extension cables

Always meet in MIL beforehand, we may spend as much as an hour of unexpected time prepping the sub.


# Spells

* Copy things to the sub

    scp <local_directory> sub8@mil-sub-sub8.ad.mil.ufl.edu:<remote_directory>

    * local_dir - The file on your local machine that you want to copy
    * remote_dir - The file that you want create on the sub (if none is specified, the file with the same name as the original will be placed in the home folder)

* Add a git remote for the sub

    git remote add sub ssh://sub8@mil-sub-sub8.ad.mil.ufl.edu:/home/sub8/<workspace>/src/<repository>

    * workspace - The name of the catkin workspace in which the repository exists
    * repository - The name of the repository that is being selected as a remote