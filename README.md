# J-Fi ROS2 Bridge

J-Fi is a ROS2 Serial Communication Interface

![image](https://github.com/user-attachments/assets/089d86cf-dab3-48db-b4c0-dfab6dbb34eb)

## Structure

- **jfi_comm** : ROS2 Serial Communication Interface to use J-Fi module
- serial_comm_node : **Example** ROS2 Node showing its usage
```
jfi_comm/
├── config/
│ ├── jfi.xml                     # Mavlink message definition.
├── include/
│ ├── jfi_comm.hpp                # Header file for the JFi Interface.
│ └── serial_comm_node.hpp        # Header file for the Example ROS2 node using Interface
├── launch/
│ ├── test_simulation.launch.py   # Example Launch file for simulation tests using virtual serial ports.
│ └── serial_comm_node.launch.py  # Example Launch file for running the node in a real environment.
├── mavlink/
└── src/
  ├── jfi_comm.cpp                # Implementation of the JFi Interface.
  ├── serial_comm_node.cpp        # Implementation of the Example ROS2 node using Interface
  └── main.cpp
```

## Features

- **MAVLink Message Handling**: Encodes ROS2 messages into MAVLink messages and decodes received MAVLink messages back into ROS2 messages.
- Provides a virtual serial port test environment with socat

## Prerequisites

- **ROS2**: Tested on ROS2 Humble/Jazzy.
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
colcon build --symlink-install
source install/setup.bash
```

## Run

### Parameters

- port_name: Serial port device path (default: /dev/ttyUSB0).
- baud_rate: Baud rate for serial communication (default: 115200).
- system_id: MAVLink system ID (default: 1).
- component_id: MAVLink component ID (default: 1).

### For simulation test,
Use virtual serial ports via socat:
```
ros2 launch  jfi_comm test_simulation.launch.py
```
Follow the example below to see if two-way communication is working properly
- Open a new terminal and use the ros2 topic pub command to send a test message
```
ros2 topic pub --once /jfi_comm/in/string std_msgs/msg/String "{data: 'Hello, World!'}"
```

### For real environment,
Use real serial ports:
```
ros2 launch jfi_comm serial_comm_node.launch.py
```
(Optional) Example with custom configuration:
```
ros2 launch jfi_comm serial_comm_node.launch.py port_name:=/dev/ttyUSB1 baud_rate:=115200 system_id:=2 component_id:=2
```
