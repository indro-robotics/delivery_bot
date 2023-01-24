# delivery_bot
Package for operating the drone delivery robot _EVA_. The control and operation is currently split into three separate packages: *delivery_bot* , *deliverybot_arduino* , and *deliverybot_simulation*. Specific instructions for the operation of each is contained within their subfolders. 
The simulation includes an RVIZ model of the bot as well as ackermann steering control and door actuation in gazebo. This simulation is still under development and will have more features added later. 

Currently, the intelrealsense cameras are NOT UTILIZED under this codebase due to hardware limitations in the computer. Future versions will fix this issue and an autonomy system will be included. 

Branch naming convention follows this syntax: <ROBOT-VERSION>-<DATE-OF-PUSH> or in the case of development branches: <ROBOT-VERSION>-dev. Developmental branches are used for testing new features and should not be installed on client robots. 



As of current, only one IntelRealSense Camera can be used for object avoidance and SLAM. A simple closed loop object avoidance can be used with both cameras, but it requires good sensor data and does not generate a map (therefore forgetting objects that exit its FOV)
## Necessary packages for cameras to post and operating procedures
Install the Intel Realsense Camera SDK using their [Github Installation Instructions](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md).

Install the ROS-Wrapper for the Intel Realsense Cameras:
```bash
sudo apt-get install ros-$ROS_DISTRO-realsense2-camera
```

For running RTABMap on the Robot, install the RTABMAP and RTABMAP ROS packages:
```bash
sudo apt install ros-$ROS_DISTRO-rtabmap ros-$ROS_DISTRO-rtabmap-ros
```
**The debian package for RTABMAP-ROS does not support multi-camera RGB parsing and syncing, need to install from source which is not suppported well on Jetson Nano **





roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/cam_front/aligned_depth_to_color/image_raw rgb_topic:=/cam_front/color/image_raw camera_info_topic:=/cam_front/color/camera_info approx_sync:=false frame_id:=/base_link rviz:=true rtabmapviz:=false


roslaunch rtabmap_ros rtabmap.launch rtabmap_args:="--delete_db_on_start" depth_topic:=/cam_rear/aligned_depth_to_color/image_raw rgb_topic:=/cam_rear/color/image_raw camera_info_topic:=/cam_rear/color/camera_info approx_sync:=false frame_id:=/base_link rviz:=true rtabmapviz:=false
