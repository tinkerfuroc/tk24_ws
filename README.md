# tk23_ws
The new workspace for 2023-2024
# Chassis Setup
Setup usb can communication:
```sh
ip link set can0 type can bitrate 1000000
ip link set up can0
```
Laucnh the interface and controller:
```sh
ros2 launch tinker_chassis_bringup tinker_bringup.launch.py
```
To send command to chassis, publish on `cmd_vel` topic.
