To run the Sub, you must install many dependencies. Here is a nice cheat sheet, copied from our Semaphore setup-build script.

This assumes you have installed ros-desktop-full and all the catkin nonsense already.

```
sudo apt-get update -qq
sudo apt-get install -qq libboost-all-dev cmake python-dev python-qt4-dev python-qt4-gl python-opengl freeglut3-dev libassimp-dev
sudo apt-get install -qq libompl-dev
sudo apt-get install -qq python-scipy python-pygame python-numpy python-serial
```