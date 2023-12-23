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

# Bardapi
https://github.com/jeffhong824/ML-DL/tree/master/AIGC

## env

python env >= 3.9

## Install
```bash
$ pip install bardapi
$ pip install git+https://github.com/dsdanielpark/Bard-API.git
$ pip install bardapi==0.1.38
```

1. **Set Your Bard API Key as an Environment Variable**: 
   Configure your Bard API key as an environment variable named `_BARD_API_KEY`. This is crucial for the script to authenticate and access the Bard API services.

2. **Adjust the `image_path` Variable**: 
   Modify the `image_path` variable in the script to point to the path of the image you wish to analyze. Ensure that the path correctly leads to your image file.

3. **Execute the Script**: 
   Run the script in your Python environment. The script will open the image file, utilize the Bard API to analyze the image, and return a scene description along with a categorized list of objects found in the image.

## Important Notes

- **Correct Image Path**: 
  Ensure that the image path specified in the `image_path` variable is accurate and points to a valid image file.

- **API Key Confidentiality and Accuracy**: 
  Make sure that your Bard API key is correctly set up in the environment variable and kept confidential. This key is essential for accessing the Bard API services.

- **Python Environment Compatibility**: 
  Verify that your Python environment meets the version requirements of the script. The script is intended to be run in Python 3.9 or higher.

# Building
## Build ORB-SLAM3
```bash
cd ~/.../3DCV_Final_Project_Group20/ORB_SLAM3
./build.sh
./build_ros.sh
```
## Build ROS Workspace
```bash
cd ~/.../3DCV_Final_Project_Group20/ros_ws
catkin_make
chmod +x src/deeplabv3/src/deeplabv3.py 
```
# Data
## Map
Please download map_mark.osa and make a new directory under ORB_SLAM3 name Map then put park_mark.osa in it

map_mark.osa is the map make by data recorded in 12/14

Google Drive Link:https://drive.google.com/file/d/1fCwGJlvWuexGKJZB9zu_nXNAE3oIJzQ5/view?usp=drive_link

```bash
cd ~/.../3DCV_Final_Project_Group20/ORB_SLAM3
mkdir Map
cd Map
cp [where your download] ./
```

## Rosbag
Please download sample rosbag 2023-12-24-00-06-13.bag (2.8GB) anywhere you what

Google Drive Link:https://drive.google.com/file/d/1qXwgiOAm4nwgloDVCRgxcMsO-JffrzrU/view?usp=sharing

# Quick start
## mapping

Make a new map by rosbag 2023-12-24-00-06-13.bag

```bash
# terminal 1
roscore
# terminal 2
cd ~/.../Final_Project/ros_ws/
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun deeplabv3 deeplabv3.py
# terminal 3
rosrun topic_tools drop /camera/color/image_raw 2 3 /camera/color/segmentation/image_raw
# terminal 4
cd ~/.../3DCV_Final_Project_Group20/ORB_SLAM3/
rosrun ORB_SLAM3 RGBD ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i_mapping.yaml
# wait until ORB-SLAM3 terminal show
# terminal 5
cd [where your download rosbag 2023-12-24-00-06-13.bag]
rosbag play 2023-12-24-00-06-13.bag
# see the result and after rosbag finish press "Stop" and "Reset" map will save in default destination(Map/new.osa)
```

## localization

Relocalization in the map make by data recorded in 12/14

```bash
# terminal 1
roscore
# terminal 2
cd ~/.../Final_Project/ros_ws/
source /opt/ros/noetic/setup.bash
source devel/setup.bash
rosrun deeplabv3 deeplabv3.py
# terminal 3
rosrun topic_tools drop /camera/color/image_raw 2 3 /camera/color/segmentation/image_raw
# terminal 4
cd ~/.../3DCV_Final_Project_Group20/ORB_SLAM3/
rosrun ORB_SLAM3 RGBD ./Vocabulary/ORBvoc.txt ./Examples/RGB-D/RealSense_D435i_localization.yaml
# wait until ORB-SLAM3 terminal show
# terminal 5
cd [where your download rosbag 2023-12-24-00-06-13.bag]
rosbag play 2023-12-24-00-06-13.bag
```
# Note
## change semantic mask classes
Open ./ros_ws/src/deeplabv3/src/deeplabv3.py
Change line 41 (default just mask class 11)

```python
self.mask = [11]
```
to
```python
self.mask = [any class id you what to delete or Bard result]
# example
# self.mask = [11,12,13,14,15,16,17,18]
```
### class table
| ID    |  class        | ID    |  class        | ID    |  class        | ID    |  class        |
| :---: | :---:     | :---:     | :---:     | :---:     | :---:     | :---:     | :---:     |
|0 |road|5 |pole|10 |sky|15 |bus|
|1 |sidewalk|6 |traffic light|11 |person|16 |train|
|2 |building|7 |traffic sign|12 |rider|17 |motorcycle|
|3 |wall|8 |vegetation|13 |car|18 |bicycle|
|4 |fence|9 |terrain|14 |truck| | |

# Code Reference

UZ-SLAMLab/ORB_SLAM3 : https://github.com/UZ-SLAMLab/ORB_SLAM3

VainF/DeepLabV3Plus-Pytorch : https://github.com/VainF/DeepLabV3Plus-Pytorch
