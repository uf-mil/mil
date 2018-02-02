#!/usr/bin/env python
import os
import re
import sys
import subprocess
import shlex

# Joseph Brooks
# Checks NaviGator for all python files

if len(sys.argv) == 1:
    print("add directory path to be searched as an argument")
    sys.exit()

directory = sys.argv[1]

allFileList = []

# gets all files in directory
for root, dirs, files in os.walk(directory):
    for name in files:
        programName = os.path.join(root, name)
        allFileList.append(programName)

pythonList = []

# attempts to open file as text and checks #!/usr/bin/env
# Just checks the first line
for program in allFileList:
    try:
        for line in open(program):
            if re.search("#!/usr/bin/env python", line):
                pythonList.append(program)
            break
    except:
        pass

# checks for ending .py
for program in allFileList:
    if '.py' in program and '.pyc' not in program:
        pythonList.append(program)

# checking for duplicates
finalList = []

for program in pythonList:
    # do not add if duplicate
    if program in finalList:
        pass
    else:
        finalList.append(program)

counter = 0
for program in finalList:
    # added in the subprocess argument
    counter = counter + 1
    bashCommand = """python2.7 -m flake8 --ignore E731
    --exclude=./deprecated,./gnc/navigator_path_planner/lqRRT,__init__.py
     --max-line-length=120 """ + program
    args = shlex.split(bashCommand)
    subprocess.call(args)
