# J-Fi ROS2 Bridge

J-Fi is a ROS2 Serial Communication Interface

![image](https://github.com/user-attachments/assets/089d86cf-dab3-48db-b4c0-dfab6dbb34eb)

## Structure

- jfi_comm : ROS2 Serial Communication Interface to use J-Fi module
- serial_comm_node : Example ROS2 Node showing its usage
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

- **MAVLink Message Handling**: Seamlessly encodes ROS2 messages into MAVLink packets and decodes them back on reception
- **Automatic Packet Fragmentation**: Reliably transmits large data by automatically handling packet fragmentation and reassembly
- **Virtual Serial Port Testing**: Includes a test environment using `socat` to simulate serial communication without physical hardware

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

- `port_name`: Serial port device path (default: /dev/ttyUSB0).
- `baud_rate`: Baud rate for serial communication (default: 115200).
- `system_id`: MAVLink system ID (default: 1).
- `component_id`: MAVLink component ID (default: 1).

### For simulation test,
Use virtual serial ports via `socat`:
```
ros2 launch  jfi_comm test_simulation.launch.py
```
Follow the example below to see if two-way communication is working properly
- Open a new terminal and use the `ros2 topic pub` command to send a test message
```
ros2 topic pub --once /jfi_comm/in/string std_msgs/msg/String "{data: 'Hello, World'}"
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

## WireShark 패킷 분석
Wireshark Lua 스크립트 생성
```
cd <path_to_mavlink>
python3 -m pymavlink.tools.mavgen --lang=WLua --wire-protocol=2.0 --output=jfi.lua /home/user/J-Fi/src/jfi_comm/config/jfi.xml
```
- 생성한 lua 파일을 Wireshark가 인식가능한 플러그인 폴더(ex: ~/.config/wireshark/plugins)로 이동
- wireshark를 이용해 패킷 분석

시리얼 포트로 통신이 이루어지는 경우 MAVProxy를 이용한 시리얼-UDP 브릿징 과정 필요
- 예시
```
pip3 install mavproxy
mavproxy.py --master=/dev/ttyUSB0 --baudrate=115200 --out=udp:127.0.0.1:14550
```