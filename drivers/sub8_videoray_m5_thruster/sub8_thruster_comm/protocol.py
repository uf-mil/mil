class Const(object):
    '''
    Bibliography:
        [1] VideoRay Example Code [Online]
            Available: https://github.com/videoray/Thruster/blob/master/thruster.py
    '''
    # VRCSR protocol defines  
    sync_request  =  0x5ff5
    sync_response =  0x0ff0
    protocol_vrcsr_header_size = 6
    protocol_vrcsr_xsum_size   = 4

    # CSR address for sending an application specific custom command
    addr_custom_command  = 0xf0
    propulsion_command = 0xaa

    # Flag for the standard thruster response which contains 
    response_thruster_standard = 0x2

    # Standard response is the device type followed by 4 32-bit floats and 1 byte
    response_thruster_standard_length = 1 + 4 * 4 + 1 