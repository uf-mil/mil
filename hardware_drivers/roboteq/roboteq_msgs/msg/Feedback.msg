# 50Hz feedback message for controls purposes
Header header

# Current flowing in the motors (A)
float32 motor_current

# Output stage, as a proportion of full (-1..1)
float32 motor_power

# Commanded and measured speed of the motors (rad/s)
# Position is reported in rads, and wraps around +-6M
float32 commanded_velocity
float32 measured_velocity
float32 measured_position

# Electrical power supply to the driver (V, A)
float32 supply_voltage
float32 supply_current

# Measured temperatures (C)
# Motor temp is processed from a thermal sensor connected to analog input 1.
float32 motor_temperature
# Channel temp is the temperature of the FETs. This is reported by the controller.
float32 channel_temperature
