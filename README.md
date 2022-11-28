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

And launch the empty world example:

`ros2 launch turtlebot3_gazebo empty_world.launch.py`

# Running the Algorithms

## Tangent Bug

Run the Gazebo Scenario:

`ros2 launch turtlebot3_gazebo turtlebot3_islands.launch.py`

And to run the algorithm, type as follows with the objective point as parameters:

`python3 PMR_ws/src/PMR/turtlebot3_gazebo/scripts/tangent_bug.py 9,-1`

## Curve Following

Start the empty world:

`ros2 launch turtlebot3_gazebo empty_world.launch.py`

Run the algorithm:

`python3 PMR_ws/src/PMR/turtlebot3_gazebo/scripts/vectorfield.py`

## Potential Fields

Run the Gazebo Scenario:

`ros2 launch turtlebot3_gazebo turtlebot3_islands.launch.py`

Run the algorithm, selecting the objective point followed by the potential function type:

`python3 PMR_ws/src/PMR/arp_controller.py 9 0 3`

## Wave-Front

Wave-Front was tested in two scenarios, a curved path and a bifurcation. The current state of the implementation is configured to addopt the biffurcation map. To change the scenario, few changes in the code will be necessary. A fixed goal is previously considered for map computation, to change it, small alterations will also be needed.

### Running Bifurcation Scene and Wave-Front

To open the scene in Gazebo, run:

`ros2 launch turtlebot3_gazebo turtlebot3_maze2.launch.py`

To run the wave-front navigation:

`python3 PMR_ws/src/PMR/wavefront.py`

### Running Maze Scene and Wave-Front

To open the scene in Gazebo, run:

`ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py`

Before running the wave-front navigation, change line 65 from `PMR_ws/src/PMR/wavefront.py` to:

`self.img = np.load(os.path.join(os.path.dirname(os.path.abspath(__file__)), 'img/wave.npy'))`

And then you can run:

`python3 PMR_ws/src/PMR/wavefront.py`

## Rapdly-exploring Random Trees

To run RRT, the path is already computed in `img/path_RRT.npy`, so all you need to do is run:

`ros2 launch turtlebot3_gazebo turtlebot3_islands.launch.py`

`python3 PMR_ws/src/PMR/turtlebot3_movement.py`