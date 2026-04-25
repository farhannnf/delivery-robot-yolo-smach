# TA2 Delivery Robot 🚀

This repository contains a ROS-based autonomous delivery robot system that integrates YOLO for person detection and SMACH for task execution.

## 📌 Overview
The robot is capable of:
- Autonomous navigation using ROS Navigation Stack
- Detecting and locating a person using YOLO
- Executing delivery tasks using a state machine (SMACH)

## 🧠 System Architecture
Main components:
- Navigation (move_base, 2D Nav Goal)
- YOLO-based perception
- Coordinate transformation
- SMACH state machine
- Position database (YAML)

## 📂 Structure
- config/ → position database
- launch/ → ROS launch files
- maps/ → environment maps
- scripts/ → main Python scripts

## ▶️ How to Run
```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ta2_farhan run.launch
