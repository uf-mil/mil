class Const(object):

    '''
    Bibliography:
        [1] VideoRay Example Code [Online]
            Available: https://github.com/videoray/Thruster/blob/master/thruster.py
    '''
    # VRCSR protocol defines
    sync_request = 0x5ff5
    sync_response = 0x0ff0
    protocol_vrcsr_header_size = 6
    protocol_vrcsr_xsum_size = 4

    # CSR address for sending an application specific custom command
    addr_custom_command = 0xf0
    propulsion_command = 0xaa

    # Flag for the standard thruster response which contain
    response_thruster_standard = 0x2
    response_ask_nothing = 0x00

    # Standard response is the device type followed by 4 32-bit floats and 1 byte
    response_thruster_standard_length = 1 + 4 * 4 + 1
    thrust_response_length = (
        protocol_vrcsr_header_size +
        protocol_vrcsr_xsum_size +
        response_thruster_standard_length +
        protocol_vrcsr_xsum_size
    )

    # Add your stupid size to this!
    response_normal_length = (
        protocol_vrcsr_header_size +
        protocol_vrcsr_xsum_size +
        protocol_vrcsr_xsum_size
    )

    # TODO: Get R/W flags
    csr_address = {
        'undervoltage_trigger': (0xa5, 1),
        'overvoltage_trigger': (0xa6, 1),
        'overcurrent_trigger': (0xa7, 1),
        'temp_trigger': (0xa8, 1),
        'stall_count_max': (0xa9, 1),
        'fault_control': (0xa4, 1),
        'fault': (0x14, 4),
        'save_settings': (0xee, 2),
        'undervoltage_err_cnt': (0xac, 4),
        'overvoltage_err_cnt': (0xb0, 4),
        'overcurrent_err_cnt': (0xb4, 4),
        'temp_err_cnt': (0xb8, 4),
        'stall_err_cnt': (0xbc, 4),
    }

    format_char_map = {
        1: 'B',  # unsigned char      integer 1
        2: 'H',  # unsigned short     integer 2
        4: 'I',  # unsigned int       integer 4
        8: 'Q',  # unsigned long long integer 8
    }


if __name__ == '__main__':
    addr, size = Const.csr_address['stall_err_cnt']
