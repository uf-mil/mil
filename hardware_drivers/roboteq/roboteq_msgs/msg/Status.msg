# 10Hz status message for informational/diagnostics purposes
Header header

uint8 FAULT_OVERHEAT=1
uint8 FAULT_OVERVOLTAGE=2
uint8 FAULT_UNDERVOLTAGE=4
uint8 FAULT_SHORT_CIRCUIT=8
uint8 FAULT_EMERGENCY_STOP=16
uint8 FAULT_SEPEX_EXCITATION_FAULT=32
uint8 FAULT_MOSFET_FAILURE=64
uint8 FAULT_STARTUP_CONFIG_FAULT=128
uint8 fault

uint8 STATUS_SERIAL_MODE=1
uint8 STATUS_PULSE_MODE=2
uint8 STATUS_ANALOG_MODE=4
uint8 STATUS_POWER_STAGE_OFF=8
uint8 STATUS_STALL_DETECTED=16
uint8 STATUS_AT_LIMIT=32
uint8 STATUS_MICROBASIC_SCRIPT_RUNNING=128
uint8 status

# Temperature of main logic chip (C)
float32 ic_temperature

# Internal supply and reference voltage (V)
float32 internal_voltage
float32 adc_voltage
