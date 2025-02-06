# J-Fi

J-Fi is a ROS2 Serial Communication Library using MAVLink.
It provides functionality for encoding, transmitting, and receiving MAVLink messages, and supports ROS2 message serialization/deserialization.

## Features

- **Serial Port Management**: Opens and closes the serial port with configurable parameters.
- **MAVLink Message Handling**: Encodes ROS2 messages into MAVLink messages and decodes received MAVLink messages back into ROS2 messages.

## Prerequisites

- **ROS2**: Tested on ROS2 Humble (or specify your version).
- **MAVLink**: Ensure that the required MAVLink libraries are installed.
- **socat**: (For simulation) Used to create virtual serial ports.


## Installation

### Clone repository
```
git clone https://github.com/SUV-Lab/J-Fi.git
cd J-Fi
git submodule update --init --recursive
```

### Build
```
colcon build
source install/setup.bash
```


## Run

For simulation test,
```
ros2 launch  jfi_comm test_simulation.launch.py
```

For real environment,
```
ros2 launch jfi_comm serial_comm_node.launch.py
```
(Optional) Example Launch Configuration
```
ros2 launch jfi_comm serial_comm_node.launch.py port_name:=/dev/ttyUSB0 baud_rate:=115200
```
