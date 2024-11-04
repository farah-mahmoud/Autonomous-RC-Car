# 1. SLAM Learning Phase

We will be learning all about *SLAM*.


## Slam Course
* Watch this [video](https://www.youtube.com/watch?v=B2qzYCeT9oQ&list=PLpUPoM7Rgzi_7YWn14Va2FODh7LzADBSm) about **SLAM COURSE (Optional)**.
* Watch this [video](https://www.youtube.com/watch?v=saVZtgPyyJQ&t=777s) about **SLAM (Pose graph optimization)**.
* Watch this [video](https://www.youtube.com/watch?v=NrzmH_yerBU) about **Particle filter**.
* Watch this [video](https://www.youtube.com/watch?v=_tVBxPhYUA4) about **ICP** 
  / Watch this another [video](https://www.youtube.com/watch?v=LETPf6eoyYg) about **ICP**.

## Pacakages and Catkin
Watch this [video](https://www.youtube.com/watch?v=xHh6kd5aQxA) about **Pacakages and Catkin**.

## TF
Watch this [video](https://www.youtube.com/watch?v=_t4HZ8r_qFM) about **TF**
  / Watch this another [video](https://www.youtube.com/watch?v=QyvHhY4Y_Y8&t=28s) about **TF**.

## Build Systems
Watch this [video](https://www.youtube.com/watch?v=rF--wOD4H8I) about **MAKE**.

## Launch Files
Watch this [video](https://www.youtube.com/watch?v=MiLDsw1RHV4&t=116s) about **Launch Files**.  

---

# 2. Gazebo Simulation & Robot Modeling

Watch this [video](https://www.youtube.com/playlist?list=PLK0b4e05LnzbsYJ5WH-S5td2aclJqpDYo) about **Gazebo Simulations**
Watch this [video](https://www.youtube.com/playlist?list=PLK0b4e05LnzYqEgKmTSgHewdsaTAoK6sJ) about **Robot Modeling**

---
# 3. Control


## Main Course
The main course for our control team will be the Coursera course titled [Introduction to Self-Driving Cars](https://www.coursera.org/learn/intro-self-driving-cars). This course includes several modules and playlists that will enhance our understanding of specific topics.

### Note:
Please start applying for financial aid for this course right away. It contains very useful projects, especially the final project.

## Week 1
### 1- Module 4 - Introduction to Self-Driving Cars
- [Coursera Course Link](https://www.coursera.org/learn/intro-self-driving-cars)

### 2- PID Controller
- Video Playlist: [Click here](https://www.youtube.com/playlist?list=PLn8PRpmsu08pQBgjxYFXSsODEF3Jqmm-y)

### 3- Module 5 - Introduction to Self-Driving Cars
- [Coursera Course Link](https://www.coursera.org/learn/intro-self-driving-cars)

## Week 2
### 1- State Space
- Video Playlist: [Click here](https://www.youtube.com/watch?v=hpeKrMG-WP0&list=PLfqhYmT4ggAtpuB1g8NbgH912PwYjn_We)
- This playlist is important for implemnting the following controllers.

### 2- MPC Controller
- Video Playlist: [Click here](https://www.youtube.com/playlist?list=PLn8PRpmsu08ozoeoXgxPSBKLyd4YEHww8)

### 3- Module 6 - Introduction to Self-Driving Cars
- [Coursera Course Link](https://www.coursera.org/learn/intro-self-driving-cars)

### 4- LQR Controller
- Video Playlist: [Click here](https://www.youtube.com/playlist?list=PLn8PRpmsu08podBgFw66-IavqU2SqPg_w)

## Implementation

### Final Project - Introduction to Self-Driving Cars
**Duration**: 10 days

### 1- First 5 Days (Research and Planning)
During the first 5 days, the team will focus on researching and planning the implementation of different algorithms for both Lateral and Longitudinal control. Each group will be assigned specific control algorithms to study, and each team member will have to document his research and outline the required steps from implementation to tuning. For Example:
**Group 1:**
- Lateral Control Algorithm: LQR Controller
- Longitudinal Control Algorithm: Stanley Algorithm

**Group 2:**
- Lateral Control Algorithm: PID Controller
- Longitudinal Control Algorithm: MPC Controller

### 2- Second 5 Days (Implementation)
The second phase of the project involves the actual implementation of the algorithms you studied during the first 5 days. This is the hands-on part where you'll apply your research and put your knowledge to work.  

---
# 4. Motion Planning Learning Phase

We will be learning all about **Motion Planning**.

## Videos on different topics (1 day)
*By the end of this milestone you shoud be familiar with different algorithms, their uses and limitations*

Watch this [video](https://www.youtube.com/watch?v=-fePRPyeKnc) about **Motion Planning Algorithms**.

Watch this [video](https://www.youtube.com/watch?v=QR3U1dgc5RE) about **A\* and RRT**.

Watch this [video](https://www.youtube.com/watch?v=-L-WgKMFuhE) about **A\***.

Watch this [video](https://www.youtube.com/watch?v=Ob3BIJkQJEw) about **RRT and RRT\***.

### Non holonomic constrains

Watch this [video](https://youtu.be/tjUVE3jto98?si=TNtxE8bjQNT28ONe) about **Dubins Curves**

Watch this [video](https://youtu.be/fAqh_cy7ePI?si=IM0dTvFdJaZyu3YR) about **Reeds-Shepp**

### Hybrid A* & RRT* with (Non holonomic constrains considered)

Read this [paper](https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf) about **Hybrid A\***. (our current algorithm)

Watch this [video](https://youtu.be/AOE3JYgC-Z4?si=BYLsi5ytgCvMQXzf) about **RRT\* with Dubins and Potential Fields**

## Motion Planning and Control (1 week)
*By the end of this milestone you shoud be able to implement the RRT algorithm and understand control basics*

### Skip first Module as it is about A* (you know it already):
Watch this Coursera [course](https://www.coursera.org/learn/modernrobotics-course4?specialization=modernrobotics)
or this one if you do not like Coursera [course](https://modernrobotics.northwestern.edu/nu-gm-book-resource/chapter-10-autoplay/#department)
This course is about:

* Randomized sampling-based planners, virtual potential fields, and nonlinear optimization - Module 2
* Linear error dynamics, stability of a feedback control system, and motion control of robots with joint velocities - Module 3 
* Motion control with joint torques, force control, and hybrid motion-force control - Module 4

## General Concepts about Path planning (1 week)
*By the end of this milestone you shoud be familiar with challenges in pathplanning and understand local, behaviour and mission planning*

### Skip already known topics:
Watch this Coursera [course](https://www.coursera.org/learn/motion-planning-self-driving-cars) about:
* The Planning Problem - Module 1
* Mapping for Planning - Module 2
* Mission Planning - Module 3
* Principles of Behaviour Planning - Module 4

## Advanced Topics (5 days)
*By the end of this milestone you should understand incremental path-planning, temporal logic, infinite horizon planning and more*

### Skip non related topics
Watch this Course from [mit](https://ocw.mit.edu/courses/16-412j-cognitive-robotics-spring-2016/)
You can view lecture videos [here](https://youtube.com/playlist?list=PLUl4u3cNGP62Bkdzwe7caTZC7soj7ZYvk&si=ggLfcDzbIoS1qk2x)

## Reinforcement Learning (optional)

Watch this [Series](https://youtube.com/playlist?list=PLaAAx-iIm7OloDTSgRL13FsdX2XwJh-U4&si=6bigQVFFYxsE3Cu7)

---
# 5. Perception
Watch this [video](https://www.coursera.org/learn/computer-vision-basics) about **Computer Vision Basics**.

Watch this [video](https://www.youtube.com/watch?v=1L0TKZQcUtA) about **Deep Learning Models for self-driving cars**.

Watch this [video](https://www.youtube.com/watch?v=yvfI4p6Wyvk) about **Lane Detection Basic idea**.

Watch this [video](https://www.youtube.com/watch?v=yqkISICHH-U) about **Tensorflow Object Detection in 5 Hours with Python**.

Watch this [video](https://www.youtube.com/watch?v=vGr8Bg2Fda8) **From Depth Maps to Point Clouds: A Hands-On Tutorial with Open3D in Python**.  

---


# FAQs:
## (When you start utilizing path-planning algorithms + coding in ROS, it is easy to get overwhelmed. We’ve all been there, and this is a compilation of all the confusing questions you might be asking)

### 1. What are the major differences between path-planning algorithms? (e.g. RRT*, A*, etc…)? 
This repo simulates different algorithms all in one place for easier comparison:
https://github.com/zhm-real/PathPlanning/blob/master/README.md

### In simple terms, what exactly is hybrid A*? What makes it any different than normal A*?
This blog link throughly describes what hybrid A* is in an easy-to-follow means: https://blog.habrador.com/2015/11/explaining-hybrid-star-pathfinding.html?m=1

## UNDERSTANDING WORKFLOW (Where does the path-planning sub-team stand in relation to other teams?)
### The General Process
After developing the core code logic in each team, integration takes place through ROS. Each sub-team has a code module that communicates with modules developed by other sub-teams through publishing and subscribing to topics.

### What topics does the path-planning module subscribe to (by which sub-team are they published)?
Path-planning algorithms subscribe to topics published by SLAM: (e.g., map, start and goal)

### Which sub-team subscribes to topics published by the path-planning module?
Control sub-team

## I am a bit lost. Where do I start?
### 1- Start by understanding your sub-team’s code logic FIRST
Do NOT overwhelm yourself by trying to comprehend the entire code at once. It is easier to understand concepts that you relate to (programming concepts) compared to working in Linux and ROS.


### 2- Get familiar with ROS
After you understand the core concepts of path-planning, start running the code in ROS, getting familiar with publishing and subscribing, etc…

### 3- DO NOT LEARN EVERYTHING SIMULTANEOUSLY
At this point, don’t try understanding all sub-teams’ code logic until you’re completely aware of what your own module does. Overwhelming yourself with new concepts simultaneously can slow down your learning curve.

### 4- You’re good to go!
There’s always a lot more to discover, but after you’ve gone through the aforementioned process, you can start learning at your own pace — less overwhelmed and more confident. Learning takes time.


