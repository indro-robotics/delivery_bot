# Deliverybot Simulation Environment
Simulating the InDro Robotics Deliverybot, an Ackermann steering vehicle with controllable storage compartment, in gazebo using `gazebo_ros2_control` , `xacro`, and `controller_manager`.

Simulation is run on `ubuntu_20.04` and `ROS2_Humble`.
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
By default, the simulation with launch Gazebo and RVIZ at boot, this can be changed in the launch file as needed. 

Start the simulation:

`ros2 launch deliverybot_gazebo simulation.launch.py`

The robot is controlled through `/cmd_vel`, commands can be sent through the terminal or through the teleop keyboard:

`ros2 launch teleop_twist_keyboard teleop_twist_keyboard`

This will allow keyboard control of the robot. The `t`, `b`, and `g` keys command the door.