# UE4_ROS_driver
## How to build
```
mkdir -p catkin_ws/src
cd catkin_ws/src
git clone https://github.com/lhc610github/UE4_ROS_driver
cd ..
catkin build
```

## How to use
```
source devel/setup.bash
roslaunch ue4_ros_drivers test.launch
```

## Configuration
`config/node.yaml:`
```
left_cam_driver:
  address: 192.168.2.160
  port: 6766
  topic: camera/left
  is_depth_img: false
right_cam_driver:
  address: 192.168.2.160
  port: 6767
  topic: camera/rigth
  is_depth_img: false
depth_cam_driver:
  address: 192.168.2.160
  port: 6768
  topic: camera/depth
  is_depth_img: true
imu_driver:
  address: 192.168.2.160
  port: 6769
  topic: imu
```