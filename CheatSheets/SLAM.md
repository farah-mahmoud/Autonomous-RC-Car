This file illustrates how to use Gazebo and Rviz with ROS2 foxy on Ubuntu focal fossa 20.04

## Installations

in terminal
```
sudo apt install ros-foxy-rmw_cyclonedds_cpp

sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3*

sudo apt install ros-foxy-slam-toolbox
```
make sure to have these in your .bashrc file
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/foxy/setup.bash
export TURTLEBOT3_MODEL=waffle  //model of the robot (you can also have the burger one)
```

 ## Packages you may need

in your workspace:
```
cd ~/ros2_ws/src
git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git

git clone -b foxy-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
```

and then
```
colcon build --symlink-install
```

in your terminal workspace execute the following:
 
 ## gazebo (This should simulate your real world map bas elzroof sa3ba ma3na4 beeb beeb)
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py \\ this will open gazebo with turtlebot3_world environment
```

in another terminal:
```
ros2 topic list \\check for the absence of /scan topic
ros2 topic info /scan \\ you should see LaserScan msg (lidar)

ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True \\this starts the navigation tool
```
## Rviz (this is actually what your robot sees)
in another terminal:

`ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True`


in another terminal:
```
ros2 run rviz2 rviz2 -d /opt/ros/foxy/share/nav2_bringup/rviz/nav2_default_view.rviz \\this launches rviz with existing configuration by the navigation toolbox found in this location
```
# you wanna move the robot and scan the place?

in another terminal:
```
ros2 run turtlebot3_teleop teleop_keyboard
```

## Here we are 
after moving the robot in the map don't forget to save the map before closing all your terminals
```
ros2 run nav2_map_server map_saver_cli -f my_map
```
you should now have "my_map.yaml" (info about your map) & "my_map.pgm" (pixels of your map where white indicates clear areas and black indicates obstacles)
