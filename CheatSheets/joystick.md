It's recommended to use a wireless USB gamepad as the bluetooth one may cause connection problems 
# Testing
connect the joystick and then test with
```
sudo apt install joystick jstest-gtk evtest
jstest-gtk
```

this tells us which controllers ROS can see:
```
ros2 run joy joy_enumerate_devices
```

# Run the simulation and move
run the simulation as usual 
```
ros2 launch articubot_one launch_sim.launch.py 
```
and move with your joystick

NOTE that there's an enable button 6 that you must press while moving acts as a safety button
and if you want turbo speed press button 7 while moving 
if you're not comfortable with it change the "joystick.yaml" file code. in the last line
```
require_enable_button: false
```
# Troubleshooting (optional)
if the above doesn't work run in the terminal:
```
ros2 topic echo /joy
```
Move your joystick around. If you see no messages, the joystick isn't configured or detected properly by joy_node.
If /joy works, continue to the next step.

run
```
ros2 topic echo /cmd_vel_joy
```
Check that teleop_twist_joy is receiving /joy and publishing to /cmd_vel_joy

# New package for joystick control
```
ros2 run joy joy_node
```
```
ros2 run joystick_teleop joystick_serial
```
# Go real
if the above works you're ready to test with the raspberry pi
