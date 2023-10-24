#!/usr/bin/env python3
import blessed


def main():
    inp = ""
    term = blessed.Terminal()
    pos = 0

    # TESTING VARS BEFORE REAL FUNCTIONS WORK
    hwtest = [
        "test 1",
        "test 2",
        "test 3",
        "test 4",
        "test 5",
        "test 6",
        "test 7",
        "test 8",
        "test 9",
        "test 10",
        "test 11",
    ]

    # ================

    machine = ["SubjuGator", "NaviGator", "Exit"]
    options = [
        "Hardware Checklist",
        "Software Checklist",
        "Rosnode Command",
        "Add Software Test",
        "Remove Software Test",
        "Back",
        "Exit",
    ]
    currMenu = machine

    hardwarePass = False
    softwarePass = False

    while inp != "q":
        maxPos = len(currMenu)
        # reprint line selected -> term.on_green"message"
        print(term.home + term.clear + term.center("Preflight Checklist\n"))

        for x in range(0, maxPos):
            if x == pos:
                print(term.blink + term.center(term.on_yellow(currMenu[x])))
            else:
                if currMenu == options:
                    # if menu is options, set hardware/software to red or green depending on if it's completed
                    if x == 0:
                        #   pass
                        print(
                            term.center(term.on_green(currMenu[x]))
                            if hardwarePass
                            else term.center(term.on_red(currMenu[x])),
                        )
                    elif x == 1:
                        print(
                            term.center(term.on_green(currMenu[x]))
                            if softwarePass
                            else term.center(term.on_red(currMenu[x])),
                        )
                    else:
                        print(term.center(currMenu[x]))

                else:
                    print(term.center(currMenu[x]))

        with term.cbreak(), term.hidden_cursor():
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
            if inp.name == "KEY_ENTER" or inp == " ":
                # if selection is exit
                if pos == maxPos - 1:
                    inp = "q"
                    continue
                # if in first screen go to second
                if currMenu == machine:
                    currMenu = options
                    machine[pos]
                # if in second menu
                elif currMenu == options:
                    # Hardware Check
                    if pos == 0:
                        hardware_test(term, hwtest)
                    # Software Check
                    if pos == 1:
                        software_test(term)
                    # ROSnode command
                    if pos == 2:
                        run_command(term)
                    # add test to the checklist
                    if pos == 3:
                        write_test(term)
                    # if back selected
                    if pos == 5:
                        currMenu = machine
                        pos = 0
                else:
                    pass


def hardware_test(term, hwtest):
    inp = ""
    while inp != "q":
        print(term.clear + term.center("Hardware Tests\n"))
        for x in hwtest:
            print(term.on_red(x))

        with term.cbreak(), term.hidden_cursor():
            inp = term.inkey()


def software_test(term):
    # this should run ros and do all the init tests
    pass


def run_command(term):
    pass


def write_test(term):
    pass


def delete_test(term):
    pass


if __name__ == "__main__":
    main()
