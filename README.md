# Autonomous Delivery Robot with YOLO Person Detection and SMACH State Machine

An autonomous delivery robot built on ROS that integrates real-time person detection using YOLO and structured task execution using a SMACH-based state machine.

---

## 📌 Overview

This system is designed to autonomously navigate an environment, detect and localize a target person using a camera and YOLO, then execute a complete delivery task through a modular state machine. The robot supports both handcrafted waypoint positioning and active scanning mode to handle dynamic environments.

---

## 🧠 System Architecture

The system is composed of five main components that work together to achieve autonomous delivery:

- **Navigation** — ROS `move_base` with 2D Nav Goal for path planning and obstacle avoidance
- **Perception** — YOLO-based real-time person detection from camera feed
- **Coordinate Transformation** — Converts detected person position from camera frame to ROS map frame
- **Decision Making** — SMACH state machine that manages task transitions and robot behavior
- **Database** — YAML-based storage for predefined delivery positions and waypoints

---

## 📂 Repository Structure

```
ta2_farhan/
├── config/        # Position database (YAML)
├── launch/        # ROS launch files
├── maps/          # Map files
├── scripts/       # Python scripts
├── CMakeLists.txt
└── package.xml
```

---

## ⚙️ Requirements

- Ubuntu 20.04
- ROS Noetic
- Python 3
- OpenCV
- Ultralytics YOLO

Install Python dependencies:

```bash
pip install ultralytics opencv-python
```

---

## ▶️ How to Run

```bash
cd <your_workspace>/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ta2_farhan run.launch
```

---

## 🔁 Setup & Reproducibility

### 1. Create Workspace

```bash
mkdir -p <your_workspace>/catkin_ws/src
cd <your_workspace>/catkin_ws
catkin_make
```

### 2. Clone Repository

```bash
cd <your_workspace>/catkin_ws/src
git clone https://github.com/farhannnf/delivery-robot-yolo-smach.git
```

### 3. Build

```bash
cd <your_workspace>/catkin_ws
catkin_make
source devel/setup.bash
```

### 4. Run

```bash
roslaunch ta2_farhan run.launch
```

### 5. Optional — Run State Machine Separately

```bash
rosrun ta2_farhan sm_new.py
```

---

## 🔍 Features

- Real-time person detection using YOLO
- Hybrid navigation supporting both handcrafted waypoints and active scanning mode
- Coordinate transformation from camera frame to ROS map frame
- Modular and extensible state machine built with SMACH
- Database-driven delivery position system using YAML configuration files

---

## 📊 Experiment

The system was evaluated across multiple delivery scenarios involving both static and dynamically positioned target persons. Performance was measured using two primary metrics: navigation accuracy (positional error along the x and y axes) and overall detection success rate.

---

## 📌 Notes

- Replace `<your_workspace>` with your actual path (e.g., `/home/user`)
- Always source the ROS environment before running any commands
- Ensure the camera topic is active and publishing before launching
- The map must be loaded prior to starting navigation

---

## 👤 Author

**Farhan Firmansyah**  
Final Year Project — Robotics & Automation
