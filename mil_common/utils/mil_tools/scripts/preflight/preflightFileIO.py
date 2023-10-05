#!/usr/bin/env python3

SubCheckList = open("SubChecklist.txt", "a+")
line = ["This is a test\n"]
SubCheckList.writelines(line)


SubCheckList.close()
