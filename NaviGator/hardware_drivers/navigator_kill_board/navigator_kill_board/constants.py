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
    "CAN_ID": "\x12",
    "REQUEST": {
        "HEARTBEAT": "\x00",
        "KILL_COMMAND": "\x01",
        "CLEAR_KILL": "\x02",
        "IS_RC": "\x03",
        "IS_AUTONOMOUS": "\x04",
        "KILL_STATE_REQUEST": "\x05",
    },
    "NO_KILL": "\x00",
    "KILLS": {
        "PORT_F_KILL": "\x01",
        "PORT_A_KILL": "\x02",
        "STBD_F_KILL": "\x04",
        "STBD_A_KILL": "\x08",
        "KILL_RF": "\x10",
        "MTBD_HARD_KILL": "\x20",
        "MTBD_HEARTBEAT": "\x40",
        "SELF_KILL": "\x80",
        "HW_KILL": "\xFF",
    },
}
