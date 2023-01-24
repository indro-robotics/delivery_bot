# delivery_bot
Package for operating the drone delivery robot _EVA_. The control and operation is currently split into three separate packages: *delivery_bot* , *deliverybot_arduino* , and *deliverybot_simulation*. Specific instructions for the operation of each is contained within their subfolders. 
The simulation includes an RVIZ model of the bot as well as ackermann steering control and door actuation in gazebo. This simulation is still under development and will have more features added later. 

Currently, the intelrealsense cameras are NOT UTILIZED under this codebase due to hardware limitations in the computer. Future versions will fix this issue and an autonomy system will be included. 

Branch naming convention follows this syntax: \<ROBOT-VERSION\>-\<DATE-OF-PUSH\> or in the case of development branches: \<ROBOT-VERSION\>-dev. Developmental branches are used for testing new features and should not be installed on client robots. 


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

 


