message_ids = dict(
    configure_output_message_format=0x09,
    configure_binary_measurement_output_rates=0x12,
    get_almanac=0x11,
    get_ephemeris=0x30,
    software_version=0x80,
    software_crc=0x81,
    ack=0x83,
    nack=0x84,
    gps_almanac_data=0x87,
    gps_ephemeris_data=0xB1,
    meas_time=0xDC,
    raw_meas=0xDD,
    sv_ch_status=0xDE,
    rcv_state=0xDF,
    subframe=0xE0,
)
message_names = dict((id_, name) for name, id_ in message_ids.iteritems())

class SendProxy(object):
    def __init__(self, send_func):
        self._send_func = send_func
    
    def __getattr__(self, name):
        return lambda data: self._send_func(chr(message_ids[name]) + data)

def dispatch(payload, obj, **kwargs):
    if len(payload) == 0:
        raise ValueError('payload must have nonzero length')
    message_name = message_names[ord(payload[0])]
    getattr(obj, message_name, lambda data, **kwargs: None)(payload[1:], **kwargs)
