# delivery_bot
Package for operating the drone delivery robot _EVA_. As of current, only one IntelRealSense Camera can be used for object avoidance and SLAM. A simple closed loop object avoidance can be used with both cameras, but it requires good sensor data and does not generate a map (therefore forgetting objects that exit its FOV)
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
# delivery_bot
Package for operating the drone delivery robot _EVA_. The control and operation is currently split into three separate packages: *delivery_bot* , *deliverybot_arduino* , and *deliverybot_simulation*. Specific instructions for the operation of each is contained within their subfolders. 
The simulation includes an RVIZ model of the bot as well as ackermann steering control and door actuation in gazebo. This simulation is still under development and will have more features added later. 

Currently, the intelrealsense cameras are **NOT UTILIZED** under this codebase due to hardware limitations in the computer. Future versions will fix this issue and an autonomy system will be included. 

Branch naming convention follows this syntax: `<ROBOT_VERSION>-<DATE_OF_PUSH>` or in the case of development branches: `<ROBOT_VERSION>-dev`. Developmental branches are used for testing new features and should not be installed on client robots. 

The version of your robot will be included in your documentation. 


## Launching Control Nodes
Nodes should launch automatically from boot. If not, all nodes can be launched using the *boot.launch* file. Use command:

 `roslaunch delivery_bot boot.launch` 
 
 to launch all nodes. 

 To launch only peripheral control node (e.g. controlling door, lights, and signals), use command:

 `roslaunch delivery_bot eve_controller.launch`

 ## Summary
 The following topics and services are used to control delivery bot peripherals.
 > Note: when using ROCOS, all these topics are prefaced with `/ros/`
 >
 |   Topic          | Description                               |
 | -------------- | ----------------------------------------- |
 | `/light_control` | ROS service controlling front lights |
 | `/door_control`   | ROS service controlling door actuation |
 | `/light_Cond` | ROS topic publishing the current state of lights |
 | `/hunter_status` | ROS topic publishing the status of the base robot |
 | `/cmd_vel` | ROS topic publishing the current twist commands to base robot |

 
### Serial Command Breakdown
For development, controlling lights is done with serial commands to the relay. The serial commands corresponding to their lights are shown below.
>Note: Serial commands have a timing component, two serial commands cannot be sent in succession less than 0.025 seconds apart. 
>
| Light | Serial Command |
| --- | -------|
| Front Right ON |  `FE 05 00 05 FF 00 88 34` |
| Front Right OFF | `FE 05 00 05 00 00 C9 C4` |
| Front Left ON | `FE 05 00 04 FF 00 D9 F4` |
| Front Left OFF | `FE 05 00 04 00 00 98 04` |
| Back Right ON | `FE 05 00 03 FF 00 68 35` |
| Back Right OFF | `FE 05 00 03 00 00 29 C5` |
| Back Left ON | `FE 05 00 02 FF 00 39 F5` |
| Back Left OFF | `FE 05 00 02 00 00 78 05` |
