Dual-Stage Viewpoint Planner incorporates two planning stages in autonomous exploration - an exploration stage for extending the boundary of the map, and a relocation stage for explicitly transiting the robot to different sub-areas in the environment. The exploration stage develops Rapidly-exploring Random Tree (RRT) and dynamically expand the RRT over replanning steps. The relocation stage maintains a graph through the mapped environment. During the course of exploration, the method transitions back-and-forth between the two stages to explore all areas in the environment.

<p align="center">
  <img src="image/system_overview.jpg" alt="Header" width="60%"/>
</p>

Please use instructions on our [project page](https://www.cmu-exploration.com/dsv-planner).

## Instruction

The repository has been tested in Ubuntu 18.04 with ROS Melodic and Ubuntu 20.04 with ROS Noetic. Follow instructions in [Autonomous Exploration Development Environment](https://www.cmu-exploration.com/) to setup the development environment. Make sure to checkout the branch that matches the computer setup, compile, and download the simulation environments.
To setup DSV Planner, install dependencies with command lines below. Replace 'distribution' with 'melodic' or 'noetic' to match the computer setup.
```bash
sudo apt update
sudo apt install ros-distribution-octomap-ros libgoogle-glog-dev libgflags-dev
```

### Clone and Compile

```bash
git clone https://github.com/HongbiaoZ/dsv_planner.git
```
In a terminal, go to the folder and checkout the correct branch. Replace 'distribution' with 'melodic' or 'noetic'. Then, compile.
```bash
cd dsv_planner
git checkout distribution
catkin_make
```

### Launch

To run the code, go to the development environment folder in a terminal, source the ROS workspace, and launch.
```bash
source devel/setup.sh
roslaunch vehicle_simulator system_garage.launch
```
In another terminal, go to the DSV Planner folder, source the ROS workspace, and launch.
```bash
source devel/setup.sh
roslaunch dsvp_launch explore_garage.launch
```
Now, users should see autonomous exploration in action. To launch with a different environment, use the command lines below instead and replace 'environment' with one of the environment names in the development environment, i.e. 'campus', 'indoor', 'garage', 'tunnel', and 'forest'.
```bash
roslaunch vehicle_simulator system_environment.launch
roslaunch dsvp_launch explore_environment.launch
```
To run DSV Planner in a [Matterport3D](https://niessner.github.io/Matterport) environment, follow instructions on the development environment page to setup the Matterport3D environment. Then, use the command lines below to launch the system and DSV Planner.
```bash
roslaunch vehicle_simulator system_matterport.launch
roslaunch dsvp_launch explore_matterport.launch
```

## Reference

- H. Zhu, C. Cao, S. Scherer, J. Zhang, and W. Wang. DSVP: Dual-Stage Viewpoint Planner for Rapid Exploration by Dynamic Expansion. IEEE/RSJ Intl. Conf. on Intelligent Robots and Systems (IROS). Prague, Czech, Sept. 2021.

## Author
Hongbiao Zhu (hongbiaz@andrew.cmu.edu)

## Credit

[catkin_simple](https://github.com/catkin/catkin_simple), [kdtree](https://github.com/ethz-asl/nbvplanner/tree/master/kdtree), [minkindr](https://github.com/ethz-asl/minkindr), [minkindr_ros](https://github.com/ethz-asl/minkindr_ros), and [volumetric_mapping](https://github.com/ethz-asl/volumetric_mapping) packages are from open-source releases.
