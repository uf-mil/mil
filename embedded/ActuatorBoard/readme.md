Actuator Board

Connection: RS485 - 9600, 8, N, 1, None

Basic theory of operation: You send it a one byte command and it responds with the (a) state of the valve after the command, (b) state of the switch after the command, (c) a known byte that you can use to verify that the actuator board is operational, or (d) a known byte that means that that the opcode you sent is not supported. You then send it a checksum (the byte you just sent XOR with 0xFF). The board then sends back the response and the checksum.

Sending bytes: send byte (command), then send byte XOR w/ 0xFF (checksum)
Receiving bytes: receive byte (response), then receive byte XOR w/ 0xFF (checksum)

Base opcodes:

0x10 ping
0x20 open valve (allow air flow)
0x30 close valve (prevent air flow)
0x40 read switch

To 'ping' board (check if board is operational): send 0x10, if operational, will reply with 0x11.

To open valve (allow air flow): send 0x20 + valve number (ex. 0x20 + 0x04 (valve #4) = 0x24 <-- byte to send), will reply with 0x01.

To close valve (prevent air flow): send 0x30 + valve number (ex. 0x30 + 0x0B (valve #11) = 0x3B <-- byte to send), will reply with 0x00.

To read switch: send 0x40 + switch number (ex. 0x40 + 0x09 (valve #9) = 0x49 <-- byte to send), will reply with 0x00 if switch is open (not pressed) or 0x01 if switch is closed (pressed).

**Valves and switches are numbered right to left (4 pin connector on right hand side), starting from 1 with a total of 12 of each.**

Testing: I used RealTerm to test, it can send the raw serial bytes and view the responses.
