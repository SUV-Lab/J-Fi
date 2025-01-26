# J-Fi

```
ROS2 Serial Communication Library using MAVLink
```

# Installation

```
0. Clone repository
- $ git clone --branch porofly --single-branch https://github.com/SUV-Lab/J-Fi.git
- $ cd J-Fi
- $ git submodule update --init --recursive
1. Apply submodule patch
- Move 'mavlink-changes.patch' file to mavlink submodule
- $ cd J-Fi/src/jfi_comm/mavlink
- $ git apply mavlink-changes.patch
2. (Optional) Define MAVLink messages
- Add MAVLink message_definition (ex.custom.xml) to J-Fi/src/jfi_comm/mavlink/message_definitions/v1.0
3. Build
- $ colcon build
- $ source install/setup.bash
```

# Run 

```
- $ ros2 launch jfi_comm serial_comm_node.launch.py
(Optional) Example Launch Configuration
- $ ros2 launch jfi_comm serial_comm_node.launch.py port_name:=/dev/ttyUSB0 baud_rate:=115200
```
