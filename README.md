# J-Fi
J-Fi is ROS2 Serial Communication Library using MAVLink


## Installation

### Clone repository
```
git clone --branch stmoon --single-branch https://github.com/SUV-Lab/J-Fi.git
```
```
cd J-Fi
```
```
git submodule update --init --recursive
```

### Build
```
colcon build
```
```
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
- $ ros2 launch jfi_comm serial_comm_node.launch.py port_name:=/dev/ttyUSB0 baud_rate:=115200
