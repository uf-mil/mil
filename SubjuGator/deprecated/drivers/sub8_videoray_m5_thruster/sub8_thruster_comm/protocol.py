# flake8: noqa
class Const(object):

    '''
    Bibliography:
        [1] VideoRay Example Code [Online]
            Available: https://github.com/videoray/Thruster/blob/master/thruster.py
    '''
    # VRCSR protocol defines
    sync_request = 0x5ff5
    sync_response = 0x0ff0
    header_size = 6
    xsum_size = 4

    # CSR address for sending an application specific custom command
    addr_custom_command = 0xf0
    propulsion_command = 0xaa

    # Flag for the standard thruster response
    response_thruster_standard = 0x2
    response_ask_nothing = 0x00

    # Standard propulsion response is the device type (1 byte) followed by 4 32-bit floats and 1 byte
    response_thruster_standard_length = 18
    thrust_response_length = header_size + xsum_size + response_thruster_standard_length + xsum_size

    # Add your stupid size to this!
    response_normal_length = header_size + xsum_size + xsum_size

    # Address, size, type (format char), and read/write status of named CSR fields
    csr_address = {
        'rpm_target':                (0x0, 4, 'f', 'RW'),
        'pwr_target':                (0x4, 4, 'f', 'RW'),
        'rpm':                       (0x8, 4, 'f', 'R'),
        'bus_v':                     (0xc, 4, 'f', 'R'),
        'bus_i':                     (0x10, 4, 'f', 'R'),
        'fault':                     (0x14, 4, 'I', 'R'),
        'temp':                      (0x18, 4, 'f', 'R'),
        'pwr_actual':                (0x1c, 4, 'f', 'R'),
        'rpm_P':                     (0x20, 4, 'f', 'R'),
        'rpm_I':                     (0x24, 4, 'f', 'R'),
        'rpm_D':                     (0x28, 4, 'f', 'R'),
        'thruster_ID':               (0x4c, 2, 'H', 'RW'),
        'control_flags':             (0x50, 1, 'B', 'RW'),
        'motor_fault_interlock':     (0x54, 4, 'I', 'RW'),
        'motor_control_flags':       (0x60, 1, 'B', 'RW'),
        'poles':                     (0x61, 1, 'B', 'RW'),
        'pwm_deadband':              (0x62, 1, 'B', 'RW'),
        'commutation_threshold':     (0x64, 4, 'f', 'RW'),
        'commutation_loss_timeout':  (0x68, 4, 'I', 'RW'),
        'startup_dutycycle':         (0x6c, 4, 'f', 'RW'),
        'startup_initial_rpm':       (0x70, 2, 'H', 'RW'),
        'startup_final_rpm':         (0x72, 2, 'H', 'RW'),
        'startup_duration':          (0x74, 4, 'f', 'RW'),
        'deadband_neg':              (0x78, 4, 'f', 'RW'),
        'deadband_pos':              (0x7c, 4, 'f', 'RW'),
        'limit_neg':                 (0x80, 4, 'f', 'RW'),
        'limit_pos':                 (0x84, 4, 'f', 'RW'),
        'slew_rate_up':              (0x88, 4, 'f', 'RW'),
        'slew_rate_down':            (0x8c, 4, 'f', 'RW'),
        'rpm_kP':                    (0x90, 4, 'f', 'RW'),
        'rpm_kI':                    (0x94, 4, 'f', 'RW'),
        'rpm_kD':                    (0x98, 4, 'f', 'RW'),
        'max_allowable_power':       (0x99, 4, 'I', 'RW'), # ? (Recently changed in some firmwares)
        'fault_control':             (0xa3, 1, 'B', 'RW'), # The addresses in this section are one
        'undervoltage_trigger':      (0xa4, 1, 'B', 'RW'), # less than what was documented by
        'overvoltage_trigger':       (0xa5, 1, 'B', 'RW'), # VideoRay becauese I discovered it to
        'overcurrent_trigger':       (0xa6, 1, 'B', 'RW'), # be wrong experimentally.
        'temp_trigger':              (0xa7, 1, 'B', 'RW'), #
        'stall_count_max':           (0xa8, 1, 'B', 'RW'), #                   - David Soto
        'undervoltage_err_cnt':      (0xac, 4, 'I', 'R'),
        'overvoltage_err_cnt':       (0xb0, 4, 'I', 'R'),
        'overcurrent_err_cnt':       (0xb4, 4, 'I', 'R'),
        'temp_err_cnt':              (0xb8, 4, 'I', 'R'),
        'stall_err_cnt':             (0xbc, 4, 'I', 'R'),
        'comms_sync1_err_cnt':       (0xd8, 4, 'I', 'R'),
        'comms_sync2_err_cnt':       (0xdc, 4, 'I', 'R'),
        'comms_headerxsum_err_cnt':  (0xe0, 4, 'I', 'R'),
        'comms_overrun_err_cnt':     (0xe4, 4, 'I', 'R'),
        'comms_payloadxsum_err_cnt': (0xe8, 4, 'I', 'R'),
        'comms_err_flag':            (0xec, 2, 'H', 'R'),
        'save_settings':             (0xee, 2, 'H', 'W'),
        'custom_command':            (0xf0, 4, 'I', 'W'),
        'FACTORY_SERVICE_DATA':      (0xf4, 4, 'I', 'R'),
        'CONFIG_DATA_SIZE':          (0xf8, 2, 'H', 'R'),
        'CONFIG_DATA':               (0xfa, 1, 'B', 'R'),
        'FIRMWARE_VERSION':          (0xfb, 1, 'B', 'R'),
        'NODE_ID':                   (0xfc, 1, 'B', 'RW'),
        'GROUP_ID':                  (0xfd, 1, 'B', 'RW'),
        'UTILITY':                   (0xfe, 2, 'H', 'W')
    }

    format_char_map = {
        1: 'B',  # unsigned char      integer 1
        2: 'H',  # unsigned short     integer 2
        4: 'I',  # unsigned int       integer 4
        8: 'Q',  # unsigned long long integer 8
    }
