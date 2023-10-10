#!/usr/bin/env python3
import blessed

inp = ""
term = blessed.Terminal()
pos = 0

machine = ["SubjuGator", "NaviGator", "Exit"]
options = [
    "Hardware Checklist",
    "Software Checklist",
    "Rosnode Command",
    "Back",
    "Exit",
]
currmenu = machine


while inp != "q":
    maxPos = len(currmenu)
    # reprint line selected -> term.on_green"message"
    print(term.home + term.clear + term.blink + term.on_green("pressed ") + repr(inp))

    print("Preflight Checklist\n")

    for x in range(0, maxPos):
        if x == pos:
            print(term.blink + term.on_green(currmenu[x]))
        else:
            print(currmenu[x])

    with term.cbreak():
        inp = term.inkey()
        if inp == "k" or inp.name == "KEY_UP":
            if pos == 0:
                pos = maxPos - 1
            else:
                pos -= 1

        if inp == "j" or inp.name == "KEY_DOWN":
            if pos == maxPos - 1:
                pos = 0
            else:
                pos += 1
        if inp.name == "KEY_ENTER":
            if pos == maxPos - 1:
                inp = "q"
                continue
            if currmenu == machine:
                currmenu = options
            elif currmenu == options:
                pass
            else:
                pass


def hardware_test(terminal):
    pass


def software_test(terminal):
    # this should run ros and do all the init tests
    pass


def run_command(terminal):
    pass
