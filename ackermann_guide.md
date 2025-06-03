### Inside `farah_ws`
to publish odom, robot state publisher, and joint states
```bash
ros2 launch race_it hardware_launch.py
````

### Inside `ros2_ws`
to move the robot and also make sure to have the arduino connected
```bash
ros2 run teleop_buffer submit
ros2 run teleop_buffer new_teleop_serial
```
### in dev
to visualize:
```bash
rviz2
```
> ✅ **Reminder:** Make sure to build and source inside each workspace before running any commands.

---

### Future Plan

All packages will be combined into a single workspace, in shaa Allah.

