# Project Overview
This project is centered around the development of an Autonomous RC Car.  
The project aims to harness cutting-edge technologies such as LiDAR, cameras, GPS, and IMU to create a fully autonomous navigation system.  
By integrating these advanced sensors with robust software algorithms for object detection, path planning, and real-time decision-making, the car will be capable of perceiving its environment, understanding complex road scenarios, and navigating safely.  
The project involves several innovative features including SLAM (Simultaneous Localization and Mapping), dynamic obstacle avoidance, and closed-loop control systems for precise maneuvering. Our team also employs simulation tools like Gazebo and RViz to develop and test algorithms in a virtual environment before real-world implementation.  
This ambitious project not only showcases the application of modern robotics and autonomous driving technology but also aims to contribute to the field by exploring new methodologies and enhancing existing ones.

# Objectives and Features
- **Autonomous Navigation:** The RC car should have the ability to independently navigate realworld roads and reach its designated destination without manual intervention.
- **Localization and Mapping:** The car will use its sensors and actuators to map the environment, providing a detailed representation of the road, including obstacles, and generating a simulation for navigation.
- **Ostacle Avoidance:** The car should detect both static and dynamic obstacles using its sensors and avoid them through software tools like ROS (Robot Operating System).
- **Object Detection:** Equipped with a camera sensor, the car will identify objects in its path and respond accordingly, such as avoiding the object or stopping in case of an emergency.

# Technologies used
- **Object Detection:** YOLO (You Only Look Once) algorithm for real-time object detection and avoidance.
- **SLAM:** For accurate mapping and localization. 
- **Path Planning:** Algorithms: like A* and Dijkstra's for determining the best routes. 
- **Simulation Tools:** Gazebo and RViz to test and visualize the system in a controlled environment. 
- **Closed-Loop System:** Uses feedback from sensors for speed control, direction monitoring, and ensuring smooth navigation.

![Gazebo](https://www.clearpathrobotics.com/assets/guides/kinetic/warthog/_images/warthog_gazebo.png)
![object detection](https://kajabi-storefronts-production.kajabi-cdn.com/kajabi-storefronts-production/file-uploads/blogs/22606/images/1446e76-f181-6047-4e73-8d8ba3c6a50e_object_detection_1.webp)

# How to run the project
- Refer to the CheatSheets directory that contains all about the simulation and hardware configuration to get started with the rc car.  
  - **Simulation.md** to launch *gazebo* alongside *rviz2* and to implement *SLAM* and *Navigation*.  
  - **joystick.md** to use the *joystick* to move your robot instead of using the keyboard.  
- Refer to the **dev_ws** which contains all the source code for the simulation files.  
- Refer to **docs** directory to find all setups and learning phase tutorials.  

# About the team **Big Hero 6** 

## Team Members

We are the *Big Hero 6* Team from the *Faculty of Engineering, Cairo University, Electrical Power Department* working on the development of an autonomous RC car for our 2025 graduation project. Our team consists of passionate and dedicated individuals with expertise in various fields that are essential to the successful implementation of this project.

### Meet our Team:

- **Noureldeen Emad** – [Team Leader]


- **Ahmed Nasser**

  
- **Eslam Gamal**


- **Farah Mahmoud** [github](https://github.com/farah-mahmoud) / [linkedin](https://www.linkedin.com/in/farahmahmoud/)


- **Sama Khaled**


- **Yahia Mohamed**


### Supervisors:
- **Dr. Mahmoud El-Naggar** – [Prof. Cairo University, Faculty of Engineering, Electrical Power Dept.]

- **Dr. Ahmed Lasheen** – [Assistant Prof. Cairo University, Faculty of Engineering, Electrical Power Dept.]
  - Project Supervisors, providing guidance and expertise in autonomous systems and robotics.



**Check out our project proposal for more** [Autonomous RC Car Graduation Project Proposal](https://drive.google.com/file/d/1CbgzzAHH4dYqfB2Jl7uqtO-rwBZz6TLr/view?usp=sharing)
