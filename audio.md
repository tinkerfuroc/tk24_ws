# Tinker audio

## Audio capture and generate
Adapted from [audio_common](https://github.com/ros-drivers/audio_common), fix the bug [#227](https://github.com/ros-drivers/audio_common/issues/227)

Move the action and msgs in sound_play to audio_common_msgs and modify the CMakeLists.txt.

### Dependencies
```sh
    sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
    sudo apt-get install festival
```

### Usage

#### Capture and Play
```sh
    ros2 launch audio_capture capture.launch.py
```
in another terminal
```sh
    ros2 launch audio_play play.launch.py
```

#### Gerneration
```sh
    ros2 run sound_play soundplay_node.py
```
in another terminal
```sh
    ros2 run sound_play say.py "hello world!"
```
Remeber to ```source``` the workspace in each terminal.

### TO DO
speech recognition