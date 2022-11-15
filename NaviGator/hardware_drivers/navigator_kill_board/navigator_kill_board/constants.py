"""
Navigator's kill board communicates over serial using the hex codes listed below.

This should be the same as the codes on:
    http://docs.mil.ufl.edu/pages/viewpage.action?spaceKey=NAV&title=Kill+Communication+Commands

The board can operate both in a request / response way, or it can be
periodically pinged and will respond with many bytes indicating the status
of all addresses.

For the ping/Async method, send PING, than continue to read the buffer
and parse the various response bytes below.

For the sync / request method, send the REQUEST byte for whatever addresses you
are interested in, then read a 1 byte response for either RESPONSE_FALSE or RESPONSE_TRUE

The computer can also command a kill (for example, if ROS notices a criticaly low battery)
by sending the COMPUTER.KILL.REQUEST and undone with COMPUTER.CLEAR.REQUEST
"""
constants = {
    "TIMEOUT_SECONDS": 8.0,  # How often board must be pinged to not set HEARTBERAT_REMOTE True
    # Note: not official documented, this is just a guess
    "RESPONSE_FALSE": b"\x00",  # True status for synchronous requests of individual addresses
    "RESPONSE_TRUE": b"\x01",  # False status for synchronous requests of individual addresses
    "PING": {"REQUEST": b"\x20", "RESPONSE": b"\x30"},
    "KILLS": [
        "OVERALL",
        "BUTTON_FRONT_PORT",
        "BUTTON_AFT_PORT",
        "BUTTON_FRONT_STARBOARD",
        "BUTTON_AFT_STARBOARD",
        # "HEARTBEAT_COMPUTER",
        "BUTTON_REMOTE",
        # "HEARTBEAT_REMOTE",
        "COMPUTER",
    ],
    "OVERALL": {  # Should be True if any of the over are True
        "REQUEST": b"\x21",
        "TRUE": b"\x10",
        "FALSE": b"\x11",
    },
    "BUTTON_FRONT_PORT": {"REQUEST": b"\x22", "TRUE": b"\x12", "FALSE": b"\x13"},
    "BUTTON_AFT_PORT": {"REQUEST": b"\x23", "TRUE": b"\x14", "FALSE": b"\x15"},
    "BUTTON_FRONT_STARBOARD": {"REQUEST": b"\x24", "TRUE": b"\x16", "FALSE": b"\x17"},
    "BUTTON_AFT_STARBOARD": {"REQUEST": b"\x25", "TRUE": b"\x18", "FALSE": b"\x19"},
    "HEARTBEAT_COMPUTER": {  # Will return True if board is not pinged by mobo often enough
        "REQUEST": b"\x26",
        "TRUE": b"\x1A",
        "FALSE": b"\x1B",
    },
    "BUTTON_REMOTE": {"REQUEST": b"\x28", "TRUE": b"\x3A", "FALSE": b"\x3B"},
    "HEARTBEAT_REMOTE": {  # Will return True if board is not pinged by controller often enough
        "REQUEST": b"\x29",
        "TRUE": b"\x3C",
        "FALSE": b"\x3D",
    },
    "COMPUTER": {  # Allows board to be killed over serial (like through ROS)
        "KILL": {"REQUEST": b"\x45", "RESPONSE": b"\x55"},
        "CLEAR": {"REQUEST": b"\x46", "RESPONSE": b"\x56"},
        "REQUEST": b"\x27",
        "TRUE": b"\x1C",
        "FALSE": b"\x1D",
    },
    "CONNECTED": {"TRUE": b"\x1E", "FALSE": b"\x1F"},
    "LIGHTS": {  # Note: YELLOW turns off GREEN and visa versa
        "OFF_REQUEST": b"\x40",
        "OFF_RESPONSE": b"\x50",
        "YELLOW_REQUEST": b"\x41",
        "YELLOW_RESPONSE": b"\x51",
        "GREEN_REQUEST": b"\x42",
        "GREEN_RESPONSE": b"\x52",
    },
    "CONTROLLER": b"\xA0",  # Signifies the start of a controller message (joysticks & buttons)
    # Immediately followed by 8 bytes: 6 joystick bytes, 2 button bytes
    # Joystick message is 3 signed ints from -2048 to 2047
    # Button message is 16 bits signifying up to 16 buttons on/off
    "CTRL_STICKS": ["UD", "LR", "TQ"],  # Up/Down, Left/Right, Torque
    "CTRL_BUTTONS": [
        "STATION_HOLD",
        "RAISE_KILL",
        "CLEAR_KILL",
        "THRUSTER_RETRACT",
        "THRUSTER_DEPLOY",
        "GO_INACTIVE",
        "START",
        "EMERGENCY_CONTROL",
    ],
    "CTRL_BUTTONS_VALUES": {  # Amount of buttons and labels will be changed in the future
        "STATION_HOLD": b"\x00\x01",  # Button 0
        "RAISE_KILL": b"\x00\x02",  # Button 1
        "CLEAR_KILL": b"\x00\x04",  # Button 2
        "THRUSTER_RETRACT": b"\x00\x10",  # Button 4
        "THRUSTER_DEPLOY": b"\x00\x20",  # Button 5
        "GO_INACTIVE": b"\x00\x40",  # Button 6
        "START": b"\x00\x80",  # Button 7
        "EMERGENCY_CONTROL": b"\x20\x00",  # Button 13
    },
}
