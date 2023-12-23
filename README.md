# 3DCV_Final_Project_Group20
# Environment
Ubuntu 20.04

ROS Noetic

cmake version 3.27.4

## Python Library 
torch == 2.0.1

opencv-contrib-python == 4.5.5.62

## C++ Library
Pangolin (Dowload and install instructions can be found at: https://github.com/stevenlovegrove/Pangolin.)

Eigen3 (Required at least 3.1.0.)

OpenCV (Required at leat 3.0.)

# ROS Installation
http://wiki.ros.org/noetic/Installation/Ubuntu

# Building

```bash
cd ~/.../3DCV_Final_Project_Group20/ORB_SLAM3
```
# Data
## Map
Please download map_mark.osa and make a new directory under ORB_SLAM3 name Map then put map_mark.osa in it

Google Drive Link:

```bash
cd ~/.../3DCV_Final_Project_Group20/ORB_SLAM3
mkdir Map
cd Map
cp [where you download] ./
```

## Rosbag
Please download sample rosbag 2023-12-24-00-06-13.bag nowwhere you what

Google Drive Link:
# Quick start
## mapping
```bash

roscore

cd 3DCV/Final_Project/ros_ws/
source devel/setup.bash
rosrun deeplabv3 deeplabv3.py

rosrun topic_tools drop /camera/color/image_raw 2 3 /camera/color/segmentation/image_raw
```
## localization
# Note
# Reference
