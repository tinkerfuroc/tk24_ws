# Navigation

Navigation is based on the [`nav2`](https://navigation.ros.org/) based on ROS2. To deploy the navigation stack, the dependencies should be installed locally first following [nav2 dependency installation](https://navigation.ros.org/getting_started/index.html).

## Packages

- `tinker_description`: The descrption files for tinker, including `.stl` file, `.urdf` file and a python launch file `display.launch.py`.









## Appendix

### Livox Lidar Setup

Livox Lidar type: Mid-360

[Official livox-ros2-driver git repo](https://github.com/Livox-SDK/livox_ros_driver2)

[User Manual and Other documents](https://www.livoxtech.com/mid-360/downloads1)

#### Troubleshooting

1. Keep the host ip address consistent with the config file of `MID360_config.json` host address.
2. Change the lidar ip address in `MID360_config.json` to the address of the lidar address, which can be got from the last two digits of serial number.
3. Livox communication is based on UDP, so the routing tables may be checked if no data received.
4. Official software [Livox Viewer 2](https://www.livoxtech.com/de/downloads) stills works for Ubuntu22.04, which can be used to check whether lidar is in function.




