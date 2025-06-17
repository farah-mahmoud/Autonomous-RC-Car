## Inside `farah_ws`
### To publish robot state publisher
```bash
ros2 launch race_it hardware_launch.py
````
### in dev (not raspberry) (rviz is the only thing you should run on dev to avoid crash)
run rviz2 to visualize:
```bash
rviz2
```
### If you have experienced issues in joints, base_link or anything  
generate `frames.pdf` for troubleshooting
```bash
ros2 run tf2_tools view_frames
```
if everything is set correctly you should see:  
![image](https://github.com/user-attachments/assets/c0f20abf-2d63-4518-beb3-4413ff08312e)

in tkio_ros you'll find the 2 main nodes containing all the logic  
one for open loop estimation
and the other calculates and publishes odom, joint states and TF feedback from arduino  
use either one.  

make sure to have the arduino connected and all your sensors (imu, ir on shaft, lidar)  

to move the robot and also make sure to have the arduino connected  
submit node is the same as teleop_twist_keyboard (manual control)  
and new_teleop_serial is what we use to send commands the the arduion whether by manual control or by the navigation stack  
```bash
ros2 run teleop_buffer submit
ros2 run teleop_buffer new_teleop_serial
```
### To start SLAM

#### first start lidar to get the scan topic (cruical for SLAM and Nav2)
```bash
ros2 run rplidar_ros rplidar_composition
```
OR
```bash
ros2 launch race_it rplidar.launch.py
```
We still have power issues so you have to randomly plug/unplug lidar and try the 2 commands till success  
Consider butying a good adapter and a power bank and also a reliabe micro USB cable for to make it work  

#### Then start SLAM
```bash
ros2 launch race_it online_asynch_launch.py
```
once you finish save the map
### To save the map inside folder (maps) to use it again for navigation

```bash
ros2 run nav2_map_server map_saver_cli -f ~/farah_ws/src/maps/my_map
````

#### To record SLAM

```bash
ros2 bag record -o slam_record /scan /tf /tf_static /odom /map /amcl_pose
```
#### Replay with

```bash
ros2 bag play slam_record
```
you can find all navigation and SLAM parameters in config folder

### For Navigation:
```bash
ros2 launch race_it nav2_bringup_launch.py
````
and give a set point on rviz for the car to move there

