Continuous Integration
======================


We use SemaphoreCI

# I'm seeing a bug or test failure on CI that I can't reproduce on my own machine

Some possible causes...

* sleeps and pauses will have unreliable behavior on CI (and anywhere). If something is timing sensitive, can you design it to use explicit triggers instead of waiting?
    * ex: "I need thrusters, poll until thruster driver is up" vs "I need thrusters, wait 10 seconds and then start"

* Make *sure* you do a `catkin_make clean` followed by a `catkin_make` and a `catkin_make run_tests` and see if your build still works locally

* Try running `apt-get update && apt-get upgrade` (Semaphore always grabs the most recent version of everything)
    * Note that you'll have to fix your pyode installation, check the Simul8 wiki page for how to do this