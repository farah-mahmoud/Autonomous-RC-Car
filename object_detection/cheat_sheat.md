
# 🧠 Object Detection on Raspberry Pi 4

This guide covers running two object detection models on a Raspberry Pi 4 using the Pi Camera V2:

- **MobileNet SSD V2** (Standalone Python)
- **YOLOv5** (ROS 2 Foxy Integration)

---

## 🔍 Model 1: MobileNet SSD V2 (320×320)

### 📚 Reference:
[Coursera: Computer Vision with Embedded Machine Learning](https://www.coursera.org/learn/computer-vision-with-embedded-machine-learning/)

### ▶️ Run the Model:
```bash
cd object_detection
python3 live_detection.py
```

---

## 🧠 Model 2: YOLOv5 with ROS 2

### 📦 Repo:
[GitHub: YOLOv5_ROS2 - You Can Leverage on ROS2](https://github.com/moksh-401-511/YOLOv5_ROS2-YOu-can-Leverage-On-ROS2/tree/main)

---

### 1️⃣ Initialize Pi Camera V2

Make sure your Pi Camera V2 is connected and working:
```bash
vcgencmd get_camera
```

Expected output:
```
supported=1 detected=1
```

---

### 2️⃣ Start the Camera Node (V4L2)

Use the `v4l2_camera` node to stream the video:
```bash
cd object_detection
ros2 run v4l2_camera v4l2_camera_node
```

---

### 3️⃣ Launch YOLOv5 Detection

Run the YOLOv5 node with your model and image topic:
```bash
ros2 launch yolov5_ros2 yolov5_ros2_node.launch.py \
  sub_topic:='/image_raw' \
  weights:='yolov5s.pt'
```

---

## 🛑 Monitor Raspberry Pi Performance

Keep an eye on throttling (power/thermal issues) and LED behavior:
```bash
watch -n 1 vcgencmd get_throttled
```

- If the **red LED blinks** or `get_throttled` returns non-zero, you may be under-volting or overheating.