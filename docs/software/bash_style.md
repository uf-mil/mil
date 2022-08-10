# Bash Style Guide (In Progress)

## Preamble
All bash scripts should have the following lines at the top of the file:
```
#!/bin/bash
set -euo pipefail
```
The first line allows the script to be executed from the terminal (e.g. `./my_script` rather than `bash my_script`) and the second line terminates the script if any command fails.

Please also observe the following additional style rules:

* Indent using 2 spaces
