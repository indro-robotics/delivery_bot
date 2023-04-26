# Deliverybot Simulation Environment
Simulating the InDro Robotics Deliverybot, an Ackermann steering vehicle with controllable storage compartment, in gazebo using `gazebo_ros2_control` , `xacro`, and `controller_manager`.

Simulation is run on `ubuntu_22.04` and `ROS2_Humble`.
## Installation
```bash
# Create a workspace folder
mkdir -p <humble_s>/src

#Clone the repository

cd <humble_ws>/src
git clone https://github.com/indro-robotics/delivery_bot.git

#Choose the ROS2 branch
cd deliverybot
git checkout ros2

#Check dependencies
cd ../..
rosdep check --from-paths src --ignore-src --rosdistro humble

#Install dependencies
rosdep install --from-paths src --ignore-src --rosdistro humble -y

#Build and source the workspace
cd <humble_ws>
colcon build
source install/setup.bash
```

## Run
Currently, there are two Simulation modes available. The `teleop_only` mode, where the deliverybot acts as a remote controlled vehicle. It will subscribe to `cmd_vel` and drive around it's world using teleop.

The second mode is the `SLAM` mode, where simulated *Intel Realsense D435* cameras generate depth data to be used in mapping. `SLAM` mode also creates RGB camera feeds visible in RVIZ.

By default, the simulation will launch into `SLAM` mode. To launch the simulation use the command.
``` bash
ros2 launch deliverybot_gazebo simulation.launch.py
```
To launch the simulation in `teleop_only` mode, pass the `teleop_only` parameter to the launch file.
``` bash
ros2 launch deliverybot_gazebo simulation.launch.py teleop_only:=true
```

The robot is controlled through `/cmd_vel`, commands can be sent through the terminal or through the teleop keyboard:

`ros2 launch teleop_twist_keyboard teleop_twist_keyboard`

This will allow keyboard control of the robot. The `t`, `b`, and `g` keys command the door.

The robot can also be controlled through a `joystick`. The joystick node is automatically started at boot, simply plug in a compatible controller (PlayStation, XBox, etc.) and use the `LEFT_STICK` forward-backward for drive and reverse commands, and the `RIGHT_STICK` left-right for steer commands. 
