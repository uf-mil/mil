# NaivGator Kill Board

## Description
This package contains a ROS node to interface with NaviGator's student designed kill board, which communicates over USB with the main computer. The board can receive input one one of 4 emergency buttons, the main computer, and a network heartbeat to trigger a disconnect of power to the thrusters.

The driver in this package (kill_board_driver.py), connects to the board over serial to pass software kill commands (from ros_alarms) to the board, notify ROS of hardware kills, and publish diagnostics information about the board to ROS. 

## Usage
The kill board driver is included in the master_board.launch file, but to run it manually on NaviGator:
```rosrun navigator_kill_board kill_board_driver.py```

### Simulation
If you do not have physical access to the kill board hardware, but would like to test the driver set the global simulation parameter true. ```rosparam set /is_simulation True```, then run the driver. The driver will simulate the serial protocol of the board and act as if the hardware is present. In simulation mode, services are provided to simulate the pressing of the emergency buttons. To activate a button, call one of the services listed below.

## Interfaces
### Parameters
| Parameter   |   Constraints |    Description      |  Default |
|--|--|--|--|
| /is_simulation |  Boolean | see Simulation section above | False |
| ~baud |  Integer | baud rate to communicate with kill board at   |   9600  |
| ~port |  String | device of kill board's USB interface |   /dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A104OWRY-if00-port0 |

### Alarms
| Alarm  |   Description |
|--|--|
| hw-kill |  broadcasts, raised if the board is killed (power is cut to thrusters) |
| kill | listener, notifies board of raise/clear to update board's software kill status | 

### Subscriptions
| Topic  |   Type | Description |
|--|--|--|
| /wrench/current | std_msgs/String |  turns NaviGator's control indicator light green in autonomous mode and yellow in rc mode |
| /network | std_msgs/Header | passes network hearbeat messages to board to prevent board's network kill | 

### Publishers
| Topic | Type | Description |
|--|--|--|
| /diagnostics | diagnostic_msgs/DiagnosticArray | publishes status of each potential kill (button, computer, network), or error message if cannot connect to board |

### Service Servers
| Service | Type | Description
|--|--|--|
| ~BUTTON<AFT_PORT/AFT_STARBOARD/FRONT_PORT/FRONT_STARBOARD> | std_srvs/SetBool | When running in simulated mode, simulate pressing / unpressing of emergency buttons | 


