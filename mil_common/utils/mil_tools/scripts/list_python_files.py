#!/usr/bin/env python3
import os
import sys

"""
Short script to print all python files in a given directory. Used to generate a list
to check pep8 formatting. A file is included in the output if it ends in '.py' or
the first line contains one of the header strings listed below.

Usage:
   rosrun mil_tools list_python_files DIRECTORY <IGNORE... IGNORE>

Directory is the directory to find all python files in.
Ignore is a list of filenames or directory names to ignore.
"""

if len(sys.argv) < 2:
    print("Please leave directory as first argument")
ignore = []
if len(sys.argv) > 2:
    ignore = sys.argv[2:]

python_files = set()
magic_headers = ["#!/usr/bin/env python", "#!/usr/bin/python"]
for root, _, files in os.walk(sys.argv[1]):
    ig = False
    for pattern in ignore:
        if root.find(pattern) != -1:
            ig = True
            break
    if ig:
        continue
    for filename in files:
        ext = os.path.splitext(filename)[1]
        filename = os.path.join(root, filename)
        ig = False
        for pattern in ignore:
            if filename.find(pattern) != -1:
                ig = True
                break
        if ig:
            continue
        if ext == ".py":
            python_files.add(filename)
            continue
        f = open(filename)
        first_line = f.readline()
        if first_line[0:-1] in magic_headers:
            python_files.add(filename)
print(" ".join(python_files))
