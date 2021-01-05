# nav_in_flatland

### 1. What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is inspired by Ronja Gueldenring's work: [drl_local_planner_ros_stable_baselines](https://github.com/RGring/drl_local_planner_ros_stable_baselines.git). Following features are included:

##### Navigation framework in simulator Flatland
* Designed a framework for autonomous navigation for lang range dynamic enviornment. The key framework is under a finite state machine of plan_manager, which contains 5 state: INIT, WAIT_GOAL, GEN_NEW_GLOBAL, REPLAN_MID, EXEC_LOCAL
*  Desined a task generator function, which provides easy service to set different tasks of enviornment for both training and testing

* Integration of ros navigation move base and flatland simulator

##### DRL training under the navigation framework in simulator Flatland
* Training in a simulator fusion of [Flatland](https://github.com/avidbots/flatland) 
  
* Setup to train a local planner with reinforcement learning approaches from [stable baselines3](https://github.com/DLR-RM/stable-baselines3.git)


### 2. References & Tutorial
* How to use flatland: http://flatland-simulator.readthedocs.io

* ros navigation stack: http://wiki.ros.org/navigation

### 3. How to use 

###### create workspace & clone

````
mkdir -P catkin_ws/src
cd catkin_ws/src
git clone https://github.com/FranklinBF/navigation_flatland.git
````

###### install forks
````
cd nav_in_flatland
rosws update
````

###### catkin_make
````
go to catkin_ws
catkin_make
````


###### prepare your python virtual env(assumed you have installed virtualenv & virtualenvwrapper)
````
mkvirtualenv --python=python3.6 arena_py3
workon arena_py3
````
If you have not virtualenv & virtualenvwrapper, please install and setup them first.

###### install stable_baseline3 
````
pip install stable-baselines3
````
This will be used for DRL local planner, please install it in your virtual env eg. arena_py3


###### install geometry2 compiled with python3 
The official ros only support python2. In order to make the $tf$ work in python3, its necessary to compile it with python3. We provided a script to automately this this
and do some additional configurations for the convenience . You can simply run it with 
````bash
./geometry2_install.sh
After that you can try to import tf in python3 and no error is supposed to be shown up.
````

##### [Quick start] simulation env and launch
````
roslaunch flatland_bringup start_flatland.launch  train_mode:=false
````
start_flatland.launch will start several other sublaunch files and some neccesary ros packages:
   1. **start simulator node**: start flatland, load robot model
   2. **start map server node**: load map, which will provide occupancy grid used for mapping functions later
   3. **start fake localization**: which will provide static tf map_to_odom, in order to have localization of the robot.
   4. **start task generator node**: which provide task generation service for rviz_plugin(Generate Task)
   5. **start plan manager node**: provide manager for robot state estimation, mapping,  global planner and local planner,  which is the key for navigation framework.
   6. **train_mode**: 
   if true, the simulator(flatland) will provide a *step_world service* and the simulator will update its simulation when he receives a *step_world service request*.
   
   if true, the plan manager will generate subgoal topic always as goal(global goal) topic.


##### [Quick start] test with DRL training 
###### In one terminnal
```bash
roslaunch flatland_bringup start_flatland.launch  train_mode:=true
```
###### In another terminal
first **activate your python3 env**, which contains stable_baseline3, geometry2
```
workon arena_py3
```
then python run the script
```
roscd flatland_local_planner_drl
python scripts/training/training_example.py
```

Hint: During 2021-01-05 and 2021-01-10, flatland_local_planner_drl package is still under the development, which means the api of the class could be drastically changed. Sorry about the inconvinience!


### 4. Structure of the packges

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
   2. mapping:
      1. costmap2D
      2. Euclean Signed Distancefield Map
      3. Topology Graph
      4. Voxgraph
      5. ...
   3. global_planner
      1. flatland_global_planner_Dijkstra
      2. flatland_global_planner_Astar
      3. flatland_global_planner_JPS(Jump point search)
      4. flatland_global_planner_KinoAstar 
      5. flatland_global_planner_Informed_RRTstar
      6. ...
   4. local_planner
      1. flatland_local_planner_drl
      2. flatland_local_planner_cardl
      3. flatland_local_planner_TEB
      4. flatland_local_planner_VFH*
      5. ...
   5. plan_manager
      1. plan_collector
      2. plan_manager
      3. plan_manager_node
   6. plan_msgs
      1. msg
         1. RobotState.msg
      2. srv
         1. Subgoal.srv
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


### 5. Navigation framework

