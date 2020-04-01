# Mapping by Sensor Fusion with IMU and Camera (RGBD and Fisheyes)

In general, this project builds mapping function on Intel realsense tracking camera T265 and depth camera D435i individually, then compares their mapping qualities. To achieve the mapping function on the depth camera D435i, the depth information from its two RGBD eyes could be fused with IMU data by applying the EKF filter. For the tracking camera T265, two fisheys could perform as stereo cameras to measure the depth, then combine with its tracking properties to build a map.



## Demo:

Close loop for realsense D453i camera

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/ezgif.com-gif-maker-1.gif?raw=true)


Depth graph from Realsense T265 Tracking camera

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/depth.png?raw=true)

loop closure for tracking camera T265 camera

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/T265.png?raw=true)


## Mapping approach:

### Depth camera D4351:
The Depth camera sensor fusion process as shown on figure 2:

1. Applied IMU to Madwick Filter library to fused IMU data
2. Applied two RGB-D eyes to RTabMap_ros library to get cloud point data
3. Fused the new imu data from step 1 and depth data from step 2 in EKF on robot_localization library to get new odom data.
4. Fused  odom data from step3 and cloud point data from the step 2 on RTabMap to build map.  
```
see file  D435i_mapping.launch
```

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/Sensor_fusion_D435i.png?raw=true)*<center>Figure 2: Senosr fuse for Depth Camera D435i</center>*

### Tracking camera T265:
1. In order to get the depth graph, we need to treat two fisheyes as stereo camera. Re-calibrate two camera from Rtabmap.
2. Build node to publish new topics for two fisheyes information base on the calibration on step 1.
3. Applied calibrated "stereo camera" to RTabMap_ros library to get cloud point data.
4. Fused odom data from trakcing camera and cloud point data from the step 2 on RtabMap to build map.

```
see file T265_mapping.launch.
```

For more detail behand the theory, please see at my [portfolio](https://luxi-huang.github.io/portfolio/HRL/).



## Getting start:
1. Follow the instruction on [IntelRealSense_ROS](https://github.com/IntelRealSense/realsense-ros) to download the ROS, SDK and Realsense_ROS package

2. Run the T265 Depth graph:
```
$ cd scripts
$ python T265_stero.py
```

3. Get Loop closure from T265 Map:
```
$ roscore
$ roscd scripts
$ python camera_info_pub.py    _url:=/home/luxi/winter_2020/v_slam/ws/src/my_realsense/config/left.yaml    image:=/camera/fisheye1/image_raw    camera_info:=/camera/fisheye1/camera_info_calib
$ python camera_info_pub.py    _url:=/home/luxi/winter_2020/v_slam/ws/src/my_realsense/config/right.yaml    image:=/camera/fisheye2/image_raw    camera_info:=/camera/fisheye2/camera_info_calib
$ roslaunch T265_mapping.launch
```

4. Getting Loop closure from the D435i Map:
```
roslaunch my_realsense D435i_mapping.launch
```


## Comparing result:
As shown on figure 3 which compares the mapping quality and the reliability of mapping. The Mapping Quality based on the information details and inclusive of map, D435i has better mapping quality than T265 when we tested on a wide hallway.  However, when tested on narrow hallways which are  smaller than 1.5m in width, the mapping would be interrupted on the D435i camera, and only the T265 camera works.  


![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/Mapping_camperision.png?raw=true)*<center>Figure 3: Comparing Maping T265 and D435i</center>*


The Figure 4 compared  odometry when two camera complete loop closure. so we can find the at the end of the loop closure, the tracking camera T265 back to its start point closer than the depth camera D435i .  

![Hierarchy](https://github.com/luxi-huang/portfolio/blob/master/img/posts/sensor_fusion/distance_compare.png?raw=true)*<center>Figure 4: Odom comparison at the end of loop closure </center>*


## Future work:

Test two camera on the real robot, try the tasks like tracking or avoiding obstables, and compare which camera performance is better.
