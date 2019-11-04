# Overview
This page describes MIL's coding style for the various languages used
in our software, tooling, and documentation. Conforming to these
standards allows our code to be consistent and easier to read.

## Bash

### Preample
All bash scripts should have the following lines at the top of the file:
```
#!/bin/bash
set -euo pipefail
```
The first line allows the script to be executed from the terminal (e.g. `./my_script` rather than `bash my_script`) and the second line terminates the script if any command fails.

Please also observe the following additional style rules:

* Indent using 2 spaces

## C++
We follow the [ROS style guide](http://wiki.ros.org/CppStyleGuide)

### Autoformating
TODO

## Python
We follow the [ROS style guide](http://wiki.ros.org/PyStyleGuide)

### Autoformating
TODO
