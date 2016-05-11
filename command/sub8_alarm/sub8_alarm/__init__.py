# flake8: noqa

from alarm_helpers import AlarmBroadcaster
from alarm_helpers import AlarmListener
from alarm_helpers import single_alarm
from . import alarms

meta_alarms_inv = {
    'kill': ('network-timeout', 'power-failure', 'battery-voltage', 'covariance-scale'),
    'notify': ('fail-log',)
}

meta_alarms = {}
for meta_alarm, sub_alarms in meta_alarms_inv.items():
    for sub_alarm in sub_alarms:
        meta_alarms[sub_alarm] = meta_alarm
