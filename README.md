

# ROS-Based Pick-and-Place Pipeline for TIAGo Robot

## Overview

This project implements a complete ROS1 pipeline for pick-and-place manipulation using the TIAGo robot in a simulated environment.

The system integrates:
- waypoint-based navigation
- AprilTag-based object detection
- collision-aware motion planning with MoveIt

The main focus is on perception and manipulation, enabling the robot to detect objects on a table, grasp them, and place them on another table following a geometric constraint.

---

## Demo

TIAGo robot performing pick-and-place in simulation:


https://github.com/user-attachments/assets/6016635f-914a-47cc-86aa-6cc7bd245013



---

## Problem

The goal is to autonomously pick objects from a table and place them on another table along a line defined by:

y = m x + q

where the coefficients are provided by a ROS service.

The system must:
- detect objects using vision (AprilTags)
- transform object poses into the robot frame
- plan collision-free trajectories
- execute grasping and placing operations

---

## System Architecture

The system is organized into three ROS nodes:

- **Node A (Coordination & Navigation)**
  - Handles waypoint-based navigation using `move_base`
  - Manages high-level task execution
  - Computes placement positions based on the line equation

- **Node B (Perception)**
  - Detects objects using AprilTags
  - Generates collision objects for planning
  - Sends manipulation goals

- **Node C (Manipulation)**
  - Executes pick-and-place operations using MoveIt
  - Plans and executes motion trajectories

Communication between nodes is implemented using custom ROS messages and actions.

---

## Pipeline

1. Navigate to predefined positions near the pick-up table  
2. Detect objects using AprilTags and transform poses into the robot frame  
3. Add detected objects and tables as collision objects in the planning scene  
4. Execute grasping using MoveIt (approach → grasp → depart)  
5. Compute placement positions along the line y = mx + q  
6. Execute placing and repeat the process  

---

## Key Design Choices

- **Waypoint-based navigation** to ensure safe movement near tables  
- **Single picking pose strategy** to simplify manipulation  
- **Head scanning routine** to improve perception coverage  
- **Collision-aware planning** for safe trajectory generation  

---

## Challenges

Key challenges encountered during development:

- **Imprecise object poses from AprilTag detection**
  - Mitigated by ensuring the robot head is stationary during detection
  - Assumed fixed object orientation on the table

- **Motion planning failures**
  - Addressed through safe intermediate configurations
  - Use of Cartesian paths for robustness

- **Simulation variability across environments**
  - Different behaviors observed between local machines and VLab
  - Improved robustness through conservative planning strategies

---

## Technologies

- ROS1  
- MoveIt  
- Gazebo  
- AprilTag detection  
- TF transformations  

---

## Environment

This project is not fully standalone. It was developed within the TIAGo simulation environment used for the course assignment and depends on external packages and simulation assets.

---

## Authors

This project was developed as part of a group assignment:

- Riccardo Fazzi  
- Matteo Baldoni  
- Luca Grigolin  

Each member contributed to different components of the system, including perception, planning, and manipulation.
