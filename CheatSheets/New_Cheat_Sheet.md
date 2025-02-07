# Project cheat sheet

## Raspberry pi4
* power the Rpi and connect it to the same network as the dev machine
* open a terminal with 2 tabs, 3 windows each
* in each window of the 3 of the First tab run 
```
ssh -o ServerAliveInterval=60 pi@192.168.xxx.12
```
* make sure of the IP address and change 'xxx' with the correct digits using angry IP
* Enter Rpi password: 01145623392

<b> Now that you have 3 opened windows connected to Rpi through SSH <b>

Run in each window
```
cd ros2_ws
```

### 1st window: 
```
ros2 launch articubot_one launch_robot.launch.py
```
### 2nd window : 
```
ros2 launch articubot_one rplidar.launch.py
```
or run the command of rplidar_composition from history
### 3rd window : 
```
ros2 run teleop_serial teleop_serial
```  

[SOLVED]  
*Change between ACM0 and ACM1 in the node script if it gives an error*

<b> Note that you should run the above run lines on the Raspberry pi4 <b>
** Make sure Lidar and Arduino are connected to the Rpi
•
## Dev machine
``` 
sudo apt update
sudo apt upgrade
```
*Make sure that you have the package downloaded in a workspace
Open the second tab (with 3 windows)
* Change directory to the workspace that contains package : `cd ur_ws`

### 1st window :
```
rviz2
```
add your components:
* Robot Model with topic `\robot_description`
* TF
* Lidar with topic `\scan`  
and change your fixed frame into `\odom` or any desired fixed frame
### 2nd window: 
```
ros2 launch slam_toolbox online_async_launch.py
```
```
slam_params_file:=./src/articubot_one/config/mapper_params_online_async.yaml
use_sim_time:=false
```
### 3rd window: 
```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
Make sure you have all plugins and dependencies installed
* Twist mux
* Navigation and nav2_bringup
* Slam_toolbox
* Etc.
