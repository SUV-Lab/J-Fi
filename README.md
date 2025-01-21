# J-Fi

```
MAVLink Transceiver using J-Fi
```

# Installation

```
1. Move 'mavlink-changes.patch' file to mavlink submodule
2. Apply patch
- cd J-Fi/src/jfi_transceiver/mavlink
- git apply mavlink-changes.patch
3. make launch directroy
- cd J-Fi/src/jfi_transceiver
- mkdir launch
4. Build
- colcon build
```

# Run 

```
ros2 run jfi_transceiver jfi_transceiver
```
