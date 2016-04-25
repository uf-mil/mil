# Sub8 Alarm message. All alarms should be published on the /alarm topic

std_msgs/Header header

# Does the alarm require an action?
bool action_required

# A brief human-readable description of the alarm
string problem_description

# A JSON problem description
string parameters

# An integer describing severity. 0 being the worst, i.e. "This must be addressed now"
uint8 severity

# indexes into table of possible system alarms maintained by the alarm node
string alarm_name

# Node of alarm origination
string node_name

# Is this just a notification to clear an alarm of this type?
bool clear
