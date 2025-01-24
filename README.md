# J-Fi

```
ROS2 Serial Communication Library using MAVLink
```

# Installation

```
1. Apply submodule patch
- Move 'mavlink-changes.patch' file to mavlink submodule
- $ cd J-Fi/src/serial_comm/mavlink
- $ git apply mavlink-changes.patch
2. (Optional) Define MAVLink messages
- Add MAVLink message_definition (ex.custom.xml) to J-Fi/src/serial_comm/mavlink/message_definitions/v1.0
3. Build
- $ colcon build
```

# Run 

```
ros2 launch serial_comm serial_comm_node.launch.py
```
