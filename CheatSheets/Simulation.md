# Getting Started
## To run the simulation and move the robot with your keyboard
in your workspace run:
```
ros2 launch articubot_one launch_sim.launch.py world:=./src/articubot_one/worlds/obstacles.world
```
in another terminal:
```
rviz2 -d src/articubot_one/config/main.rviz
```
in another terminal:
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
## SLAM
in another terminal:
```
ros2 launch slam_toolbox online_async_launch.py params_file:=./src/articubot_one/config/mapper_params_online_async.yaml use_sim_time:=true
```
now return back to rviz2 and add a map and set the topic to ``` /map ```\
change your fixed frame from ```/odom``` to ```/map``` this just means that the robot will move and the map will be fixed.\
Move your robot around in the map until you get the full map while black means obstacles, white means free space and grey means uncertainity.
Running ``` ros2 service list``` will show you all the active services running by SLAM toolbox and any other toolboxes.\
## To save the map:
 Go ahead into rviz2 and add a new plugin called ```SLAMToolboxPlugin``` and then save the map for future editing or for navigation later on. (give your map a name and then click on save map and serialize map)\
To clarify, the .yaml and .pgm together are the "old" format.\
The PGM contains the actual cell occupancy data\
while the YAML contains metadata such as the grid resolution and origin location.\
For mapping -> change mode in "mapper_params_online_async.yaml" file to "mapping"
and for localization -> change mode to "localization" and type the directory to your previously serialized map.
## For Autonomous Navigation:

in another terminal run:
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true
```
or better
```
ros2 launch articubot_one navigation_launch.py use_sim_time:=true
```
in rviz2 set the topic of the map to "/global_costmap"\
you may wanna also set the color scheme to "costmap"