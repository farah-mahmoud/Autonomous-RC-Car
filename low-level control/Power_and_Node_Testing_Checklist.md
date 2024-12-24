
# Power and Node Testing Checklist

## 1) Check for Power Issues

### Check Logs for Undervoltage Warnings
Run the following command on the Raspberry Pi to check for undervoltage warnings:
```bash
dmesg | grep -i voltage
```
If you see warnings like `Under-voltage detected`, the power supply is insufficient.

### LED Indicators on the Pi
- A blinking red LED or no LED activity indicates power issues.
- A solid red LED means power is stable.

---

## 2) Confirm Required Packages and Dependencies
On the development machine, ensure all required packages and dependencies are installed, including:
- `twist_mux`
- `navigation`
- `nav2_bringup`
- `slam_toolbox`

---

## 3) Test Teleop Commands

While running `teleop_twist_keyboard`, check the `/cmd_vel` topic to see if velocity commands are being published:
```bash
ros2 topic echo /cmd_vel
```
- If commands are being published, verify the motor controller's response to these commands.
- Use diagnostic tools or logs to ensure the commands are reaching the hardware.

---

## 4) Verify the Node is Running

### Check the Node List
Use the following command to check if the `serial_motor number_publisher` node is listed:
```bash
ros2 node list
```
- If it’s not listed, the node isn't running. Restart the node and check again.

### Inspect the Node’s Code or Logs
- Check the source code or launch file for `serial_motor number_publisher` to see if it should produce output or initialize logging.
- Verify that it has access to the correct serial port (`/dev/ttyACM0` or `/dev/ttyACM1`).
- Look at logs using the command:
```bash
ros2 node info <node_name>
```
This will show the topics, services, and parameters associated with the `serial_motor number_publisher` node.

---

## 5) Locate the Source Code

### Find the Package Path
To locate the `serial_motor` package, use the following command:
```bash
ros2 pkg prefix serial_motor
```
This will give you the path to the package's installation directory.

### Locate the Source Directory
- If you have the source workspace, navigate to the source folder (e.g., `src/serial_motor`) in your ROS 2 workspace.
- If the source is not available, you may need to download it from your version control system (e.g., GitHub, GitLab) or contact your project administrator.

### Find the Node Script or Executable
- Look for a Python script (`.py`) or C++ file (`.cpp`) that matches the node name (`number_publisher`).
  - For Python: Check `src/serial_motor/`.
  - For C++: Check `src/serial_motor/src/`.
- To find the file, use the following command:
```bash
find src/serial_motor -name "number_publisher"
```

---

## 6) Debug Serial Communication

### Check if the Raspberry Pi Detects the Arduino or Motor Controller
Run the following command to check if the Raspberry Pi detects the Arduino or motor controller:
```bash
ls /dev/tty*
```
- You should see `/dev/ttyACM0` or `/dev/ttyACM1`. If not, confirm the hardware connection and drivers.

Use `dmesg | grep tty` to see if the device is recognized when plugged in.

---

## 7) Manually Publish Test Commands

If the node is running but the car doesn't move, manually publish a velocity command to the `/cmd_vel` topic to see if the motor responds:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```
- This should make the robot move if the motors are properly configured.
