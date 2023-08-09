## Tinker-vision (Realsense D435i)
### Driver
- install ROS2 humble: https://docs.ros.org/en/humble/index.html (no bugs)
- install driver: reference(https://github.com/IntelRealSense/realsense-ros#installation-instructions) (no bugs)
    - SDK 2.0: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages (see Installing the packages:)
        
        if success:
        ~~~
        realsense-viewer
        ~~~
        you will see a window (make sure the camera is connected)
        ![Alt text](image.png)

    - ROS2 wrapper: from source https://github.com/IntelRealSense/realsense-ros#installation-instructions
    - source environment: 
        ~~~
        . install/local_setup.bash
        ~~~
    - check:

        RGB: 
        ~~~
        ros2 run image_view image_view image:=/camera/color/image_raw
        ~~~
        Depth:

### Object detection
- opencv: 
    ~~~ 
    pip install opencv-python 
    ~~~
- YOLOv8: https://github.com/ultralytics/ultralytics
    tutorial for beginner: https://www.freecodecamp.org/news/how-to-detect-objects-in-images-using-yolov8/
