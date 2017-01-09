string alarm_name             # Name of the alarm.
bool raised                   # Is the alarm raised or not?

string node_name              # Name of the last node to raise/clear the alarm.
string problem_description    # A brief human-readable description of the alarm.
string parameters             # A JSON problem description.
uint8 severity                # How severe is the alarm? 0 is the most severe.
