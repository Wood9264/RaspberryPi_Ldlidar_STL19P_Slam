
# 操作指南

## get ldrobot lidar package
```bash
cd ./src

git clone  https://github.com/ldrobotSensorTeam/ldlidar_stl_ros.git
# 或者
git clone  https://gitee.com/ldrobotSensorTeam/ldlidar_stl_ros.git

```

## install depend and build
```bash
cd ./

sudo apt-get install ros-`rosversion -d`-gmapping

rosdep install --from-paths src --ignore-src  -r -y

catkin_make

```
## modify usb device connect pression
> please read [lidar ros package use introduction](src/ldlidar_stl_ros/README.md)
```bash
sudo chmod 777 /dev/ttyUSB* 
```


## run

```bash
cd ./

source devel/setup.bash

roslaunch demo_gmapping_slam   gmapping_ldlidar.launch
```

