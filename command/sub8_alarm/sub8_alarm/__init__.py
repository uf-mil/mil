# flake8: noqa

from alarm_helpers import AlarmBroadcaster
from alarm_helpers import AlarmListener
from alarm_helpers import single_alarm
from . import alarm_handlers
from alarm_handlers._template import HandlerBase

meta_alarms_inv = {
    'kill': ('network-loss', 'power-failure', 'battery-voltage', 'odom-loss', 'height-over-bottom'),
}

meta_alarms = {}
for meta_alarm, sub_alarms in meta_alarms_inv.items():
    for sub_alarm in sub_alarms:
        meta_alarms[sub_alarm] = meta_alarm
