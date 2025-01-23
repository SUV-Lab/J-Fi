# J-Fi

```
ROS2 Serial Communication Library using MAVLink
```

# Installation

```
1. Apply submodule patch
- Move 'mavlink-changes.patch' file to mavlink submodule
- $ cd J-Fi/src/jfi_transceiver/mavlink
- $ git apply mavlink-changes.patch
2. (Optional) Define MAVLink messages
- Add MAVLink message_definition (ex.custom.xml)
3. Build
- $ colcon build
```

# Run 

```
ros2 launch jfi_transceiver <launch file>
```
