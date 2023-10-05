#!/usr/bin/env python3
import blessed

inp = ""
term = blessed.Terminal()
pos = 0

while inp != "q":
    maxPos = 3
    height, width = term.height, term.width

    # reprint line selected -> term.on_green"message"

    print(term.home + term.clear + term.blink + term.on_green("pressed ") + repr(inp))

    print("testing screen\n")
    print("checklist")
    print("hiiardware checklist")

    print(term.move_y(term.height // 2))
    print(term.black_on_darkkhaki(term.center("press any key")))

    with term.cbreak():
        inp = term.inkey()
        if inp == "k" or inp == "KEY_UP":
            if pos == 0:
                pos = maxPos
            else:
                pos -= 1

        if inp == "j" or inp == "KEY_DOWN":
            if pos == maxPos:
                pos = 0
            else:
                pos += 1
