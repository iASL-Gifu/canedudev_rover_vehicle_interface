# CANEDUDEV_INTERFACE_FOR_ROS 2_AUTOWARE

## Requirements
Autoware Core/Universe

ROS2_Socketcan
https://github.com/autowarefoundation/ros2_socketcan/tree/1.1.0

```bash
sudo apt install can-utils
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 125000
```