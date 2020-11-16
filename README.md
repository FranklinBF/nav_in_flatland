# navigation_flatland

### create workspace & clone

````
mkdir -P catkin_ws/src
cd catkin_ws/src
git clone https://github.com/FranklinBF/navigation_flatland.git
````

### install forks
cd navigation_flatland
rosws update

### catkin_make
go to workspace
catkin_make

### start and launch
roslaunch flatland_bringup/launch/start_flatland.launch 
