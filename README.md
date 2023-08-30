# tk23_ws
The new workspace for 2023-2024
# Chassis Setup
Setup usb can communication:
```sh
sudo chmod +x ./setupcan.sh
./setupcan.sh
```
Launch the interface and controller:
```sh
ros2 launch tinker_chassis_bringup tinker_bringup.launch.py
```
To send command to chassis, publish on `cmd_vel` topic.
