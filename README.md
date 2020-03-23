# Sensor-Fusion-Realsense-Camera

This project build mapping function on Intel realsense traking camera T265 and depth camera D435i individually, then compare their mapping qualities. To achieve the mapping function on the depth camera, the depth information from its two RGBD eyes could be fused with IMU data by applying EKF filter. On the tracking camera, two fisheys could perform as stereo camera to measures the depth, then combine with its depth properties to achieve its functionalities.

## Demo:
![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/ezgif.com-gif-maker-1.gif?raw=true)*<center>Figure 1: Loop closeure for Depth Camera D435i</center>*

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/depth.png?raw=true)*<center>Figure 2: Depth graph from T265</center>*

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/T265.png?raw=true)*<center>Figure 3: loop closture from T265</center>*

## Getting start:
1. Follow the instruction on [IntelRealSense_ROS](https://github.com/IntelRealSense/realsense-ros) to download the ROS, SDK and Realsense_ROS package,

2. Run the T265 Depth graph:
```
$ cd scripts
$ python T265_stero.py
```

3. Get Loop closure from T265 Map:
```
$roscore
$roscd scripts
$ python camera_info_pub.py    _url:=/home/luxi/winter_2020/v_slam/ws/src/my_realsense/config/left.yaml    image:=/camera/fisheye1/image_raw    camera_info:=/camera/fisheye1/camera_info_calib
$ python camera_info_pub.py    _url:=/home/luxi/winter_2020/v_slam/ws/src/my_realsense/config/right.yaml    image:=/camera/fisheye2/image_raw    camera_info:=/camera/fisheye2/camera_info_calib
$ roslaunch T265_mapping.launch
```

4. Getting Loop closure from the D435i Map:
```
roslaunch my_realsense D435i_mapping.launch
```

For more information, please see at my [portfolio](https://luxi-huang.github.io/portfolio/HRL/).


## Future work:

Test two camera on the real robot, try the task like grab project or avoid obstables, and compare which camera performance is better.
