**WARNING: Running MIL software on anything other than the OS it was developed for is unsupported.**


This writeup is for those of you who wish to work with ROS on an unsupported OS (I'm looking at you, OS X). These are just a few random notes for dealing with issues that I have come across. 


__Mac OS X:__
* On OS X, GCC is symlinked to CLANG which has its own set of problems, most notably that it doe __NOT__ support OpenMP as of version 7.0. Simply running 'gcc --version' will show the following:

     >Apple LLVM version 7.0.2 (clang-700.1.81)

     >Target: x86_64-apple-darwin15.3.0

     >Thread model: posix

 This means that you will have to use your preferred package manager to pull down GCC. To build with GCC you will have to run the following command:
     
     `catkin_make -DCMAKE_C_COMPILER=/usr/local/bin/gcc-5 -DCMAKE_CXX_COMPILER=/usr/local/bin/g++-5 ...`

This is assuming that GCC is placed in /usr/local (Homebrew default)