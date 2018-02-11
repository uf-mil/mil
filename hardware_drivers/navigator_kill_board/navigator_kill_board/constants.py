'''
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
'''
constants = {
    'TIMEOUT_SECONDS': 8.0,  # How often board must be pinged to not set REMOTE to True
                             # Note: not official documented, this is just a guess
    'RESPONSE_FALSE': '\x00',  # True status for synchronous requests of individual addresses
    'RESPONSE_TRUE': '\x01',   # False status for synchronous requests of individual addresses

    'PING': {
        'REQUEST': '\x20',
        'RESPONSE': '\x30'
    },

    'KILLS': ['OVERALL', 'BUTTON_FRONT_PORT', 'BUTTON_AFT_PORT', 'BUTTON_FRONT_STARBOARD',\
              'BUTTON_AFT_STARBOARD', 'REMOTE', 'COMPUTER'],
    'OVERALL': {  # Should be True if any of the over are True
        'REQUEST': '\x21',
        'TRUE': '\x10',
        'FALSE': '\x11'
    },
    'BUTTON_FRONT_PORT': {
        'REQUEST': '\x22',
        'TRUE': '\x12',
        'FALSE': '\x13'
    },
    'BUTTON_AFT_PORT': {
        'REQUEST': '\x23',
        'TRUE': '\x14',
        'FALSE': '\x15'
    },
    'BUTTON_FRONT_STARBOARD': {
        'REQUEST': '\x24',
        'TRUE': '\x16',
        'FALSE': '\x17'
    },
    'BUTTON_AFT_STARBOARD': {
        'REQUEST': '\x25',
        'TRUE': '\x18',
        'FALSE': '\x19'
    },
    'REMOTE': {  # Will return True if board is not pinged often enough
        'REQUEST': '\x26',
        'TRUE': '\x1A',
        'FALSE': '\x1B'
    },
    'COMPUTER': {  # Allows board to be killed over serial (like through ROS)
        'KILL': {
            'REQUEST': '\x45',
            'RESPONSE': '\x55'
        },
        'CLEAR': {
            'REQUEST': '\x46',
            'RESPONSE': '\x56'
        },
        'REQUEST': '\x27',
        'TRUE': '\x1C',
        'FALSE': '\x1D'
    },
    'CONNECTED': {
        'TRUE': '\x1E',
        'FALSE': '\x1F'
    },
    'LIGHTS': {  # Note: YELLOW turns off GREEN and visa versa
        'OFF_REQUEST': '\x40',
        'OFF_RESPONSE': '\x50',
        'YELLOW_REQUEST': '\x41',
        'YELLOW_RESPONSE': '\x51',
        'GREEN_REQUEST': '\x42',
        'GREEN_RESPONSE': '\x52',
    },
    'CONTROLLER': '\xA0',  # Signifies the start of a controller message (joysticks & buttons)
			   # Immediately followed by 8 bytes: 6 joystick bytes, 2 button bytes
			   # Joystick message is 3 signed ints from -2048 to 2047
			   # Button message is 16 bits signifying up to 16 buttons on/off
    'CTRL_STICKS': ['UD', 'LR', 'TQ'],  #Up/Down, Left/Right, Torque
    'CTRL_BUTTONS': ['X', 'Y', 'A', 'B', 'DL', 'DR', 'START'],
    'CTRL_BUTTONS_VALUES': {  # Amount of buttons and labels will be changed in the future
		  # Currently mimic xbox controller labels and numbering
	'X': 0x0004, # Button 2,    	 or 0b0000000000000100
	'Y': 0x0008, # Button 3,    	 or 0b0000000000001000
	'A': 0x0001, # Button 0,    	 or 0b0000000000000001
	'B': 0x0002, # Button 1,    	 or 0b0000000000000010
	'DL': 0x0800, # Dpad Left (11),  or 0b0000100000000000
	'DR': 0x1000, # Dpad Right (12), or 0b0001000000000000
	'START': 0x0080, #Start (7),     or 0b0000000010000000
}
