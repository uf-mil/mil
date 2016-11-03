# flake8: noqa

from alarm_helpers import AlarmBroadcaster
from alarm_helpers import AlarmListener, AlarmListenerTx
from alarm_helpers import single_alarm
from . import alarm_handlers
from alarm_handlers._template import HandlerBase

meta_alarms_inv = {
    'kill': ('network-timeout', 'power-failure', 'battery-voltage', 'covariance-scale'),
    'notify': ('fail-log',)
}

meta_alarms = {}
for meta_alarm, nav_alarms in meta_alarms_inv.items():
    for nav_alarm in nav_alarms:
        meta_alarms[nav_alarm] = meta_alarm
