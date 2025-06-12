### Inside `farah_ws`
to publish odom, robot state publisher, and joint states
```bash
ros2 launch race_it hardware_launch.py
````
in tkio_ros you'll find the 2 main nodes containing all the logic
one for open loop estimation
and the other uses only imu for feedback from arduino

make sure to have the arduino connected

to move the robot and also make sure to have the arduino connected
submit node is the same as teleop_twist_keyboard (manual control)
and new_teleop_serial is what we use to send commands the the arduion whether by manual control or by the navigation stack
```bash
ros2 run teleop_buffer submit
ros2 run teleop_buffer new_teleop_serial
```
### in dev (not raspberry)
run rviz2 to visualize:
```bash
rviz2
```
> ✅ **Reminder:** Make sure to build and source inside each workspace before running any commands.

---
### for navigation alongside SLAM

```bash
ros2 launch race_it navigation_launch.py
````
you can find all navigation parameters in config folder
