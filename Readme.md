### Clone and Compile ###
```bash
cd <<dynamic_rrt_based_planner_workspace>>/src
git clone https://github.com/HongbiaoZ/dynamic_rrt_based_exploration_planner.git
cd ..
catkin_make
```
### Launch ###
###To be done###
To use this planner, first launch ground_based_autonomy_development_environment:
```bash
roslaunch vehicle_simulator system.launch
```
Then launch planner:
```bash
roslaunch drrt_planner_interface exploration.launch
```

### Who do I talk to? ###
Hongbiao Zhu (hongbiaz@andrew.cmu.edu)



