# MIL's Pneumatic Board

The pneumatic board is responsible for controlling several valves used to power
various systems. The board was created by Daniel Volya, a former MIL member, who
is now on his way to achieveing a PhD.

## Raw Communication
To send bytes: Send the command (as a byte), then send the byte XOR with `0xFF` (checksum).
To receive bytes: Receive the byte (response), then receive the byte XOR with `0xFF` (checksum).

The opcodes used by the board include:
|  Opcode  |  Command                                    |
|:---------|--------------------------------------------:|
|  `0x10`  |  Send a ping to the board                   |
|  `0x20`  |  Open valve (to allow air flow)             |
|  `0x30`  |  Close valve (to disallow air flow)         |
|  `0x40`  |  Read a switch                              |

### Intended Operations
* To 'ping' board (check if board is operational): send 0x10, if operational,
  will reply with 0x11.
* To open valve (allow air flow): send 0x20 + valve number
  (ex. 0x20 + 0x04 (valve #4) = 0x24 <-- byte to send), will reply with 0x01.
* To close valve (prevent air flow): send 0x30 + valve number
  (ex. 0x30 + 0x0B (valve #11) = 0x3B <-- byte to send),will reply with 0x00.
* To read switch: send 0x40 + switch number
  (ex. 0x40 + 0x09 (valve #9) = 0x49 <-- byte to send), will reply with 0x00
  if switch is open (not pressed) or 0x01 if switch is closed (pressed).

## Interface through ROS

In 2018, Kevin Allen, a former MIL member developed a bridge between ROS and the board.
The class responsible for this interface is {class}`mil_pneumatic_actuator.PnuematicActuatorDriver`.

This class provides several methods for completing all operations needed to control
the board.
