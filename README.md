# nav_in_flatland

### What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is inspired by Ronja Gueldenring's work: drl_local_planner_ros_stable_baselines. Following features are included:

* Setup to train a local planner with reinforcement learning approaches from [stable baselines3](https://github.com/DLR-RM/stable-baselines3.git)

* Training in a simulator fusion of [Flatland](https://github.com/avidbots/flatland) 


### References 
* How to use flatland: http://flatland-simulator.readthedocs.io
* drl_local_planner_ros_stable_baselines: https://github.com/RGring/drl_local_planner_ros_stable_baselines.git
* ros navigation stack: http://wiki.ros.org/navigation

### How to use 

#### create workspace & clone

````
mkdir -P catkin_ws/src
cd catkin_ws/src
git clone https://github.com/FranklinBF/navigation_flatland.git
````

#### install forks
````
cd nav_in_flatland
rosws update
````

#### catkin_make
````
go to catkin_ws
catkin_make
````
#### install geometry2 compiled with python3 
The official ros only support python2. In order to make the $tf$ work in python3, its necessary to compile it with python3. We provided a script to automately this this
and do some additional configurations for the convenience . You can simply run it with 
````bash
./geometry2_install.sh
After that you can try to import tf in python3 and no error is supposed to be shown up.
````


### Quick start simulation env and launch
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=false
````

### Quick test with DRL training 
In one terminnal
```bash
roslaunch flatland_bringup start_flatland.launch  train_mode:=true
```
In another terminal
```
roscd flatland_local_planner_drl
python scripts/training/training_example.py
```

Hint: During 2021-01-05 and 2021-01-10, flatland_local_planner_drl package is still under the development, which means the api of the class could be drastically changed. Sorry about the inconvinience!




### Structure of the packges

1. flatland_bringup: 
   1. config
      1. config_movebase
      2. config_plan_manager
   2. launch
      1. sublaunch:
         1. flatland_simulator.launch
         2. fake_localization.launch
         3. plan_manager.launch
         4. move_base.launch
         5. task_generator.launch
      2. start_flatland.launch
   3. rviz
2. flatland_navigation:
   1. fake_localization 
   2. global_planner
   3. local_planner
   4. plan_manager
   5. plan_msgs
3. simulator_setup
   1. maps
   2. obstacles
   3. robot
4. task_generator
   1. task_generator
      1. obstacles_manager.py
      2. robot_manager.py
      3. tasks.py
      4. utils.py
   2. scripts
      1. task_generator_node.py
5. utils
   1. rviz_plugin
   2. plan_visualization
