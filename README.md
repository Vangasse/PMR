# Setup for Movement Planning Tasks

## WSL2 for Windows 11

To install run:
`wsl --install`

To check the version:
`wsl -l -v`

For more, access: [WSL Instalation](https://docs.microsoft.com/en-us/windows/wsl/install)

## ROS2-Foxy

To install RO2 Foxy, follow the steps in: [ROS2 Foxy Instalation](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)

### Colcon

After installing ROS2, you might still need to instal colcon: [Colcon Instalation](https://colcon.readthedocs.io/en/released/user/installation.html
)

## TurtleBot3

To setup Turtlebot3 packages, run the following as in section (3.1.3) in [Turtlebot3 Quick Start](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/):

`sudo apt-get install ros-foxy-gazebo-*`

`sudo apt install ros-foxy-cartographer`

`sudo apt install ros-foxy-cartographer-ros`

`sudo apt install ros-foxy-navigation2`

`sudo apt install ros-foxy-nav2-bringup`

## Runnung Simulations

Finally, clone this repository and run:

`cd ~/PMR_ws && colcon build --symlink-install`

`source PMR_ws/install/setup.bash`

`export TURTLEBOT3_MODEL=waffle`

And launch the simulation scenarios:

`ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py`

`ros2 launch turtlebot3_gazebo turtlebot3_bamboomaze.launch.py`

<img src="https://github.com/ROBOTIS-GIT/emanual/blob/master/assets/images/platform/turtlebot3/logo_turtlebot3.png" width="300">
