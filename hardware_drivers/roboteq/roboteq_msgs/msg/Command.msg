
# Control whether the setpoint represents a velocity or position command.
# Velocity is the default, but position may be used for fine-grained movements,
# or active braking on an incline. Stopped is high-z (coasting).
int8 MODE_STOPPED=-1
int8 MODE_VELOCITY=0
int8 MODE_POSITION=1
int8 mode

# Commanded velocity (rad/s) or position (rad) for the motor. Position uses
# the same scale as measured_position in the Feedback message.
float32 setpoint
