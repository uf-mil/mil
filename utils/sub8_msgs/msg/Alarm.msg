# Sub8 Alarm message. All alarms should be published on the /alarm topic

# Does the alarm require an action?
bool action_required

# A brief human-readable description of the alarm
string problem_description

# An integer describing severity. 0 being the worst, i.e. "This must be addressed now"
uint8 severity

# indexes into table of possible system alarms maintained by the alarm node
uint8 problem_flag

# Node of alarm origination
string node_id
