# navigation_flatland

# What is this repository for?
Train DRL agents on ROS compatible simulations for autonomous navigation in highly dynamic environments. Flatland-DRL integration is based on Ronja Gueldenring's repo: drl_local_planner_ros_stable_baselines. Following features are included:

* Setup to train a local planner with reinforcement learning approaches from [stable baselines](https://github.com/hill-a/stable-baselines)

* Training in a simulator fusion of [Flatland](https://github.com/avidbots/flatland) and [pedsim_ros](https://github.com/srl-freiburg/pedsim_ros)

* Local planner has been trained on static and dynamic obstacles: [video](https://www.youtube.com/watch?v=nHvpO0hVnAg)

### Documentation ###
* How to use flatland: http://flatland-simulator.readthedocs.io
* 
### create workspace & clone

````
mkdir -P catkin_ws/src
cd catkin_ws/src
git clone https://github.com/FranklinBF/navigation_flatland.git
````

### install forks
````
cd navigation_flatland
rosws update
````

### catkin_make
````
go to catkin_ws
catkin_make
````

### quick start and launch
````
roslaunch flatland_bringup start_flatland.launch 
````

### use task generator to spawn random obstacles
````
rosrun flatland_task task_generator.py 
````


