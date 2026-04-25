# TA2 Delivery Robot 🚀

This repository presents a ROS-based autonomous delivery robot that integrates YOLO for person detection and SMACH for task execution.

---

## 📌 Overview

The system is designed to:
- Perform autonomous navigation using ROS Navigation Stack
- Detect and localize a person using YOLO
- Execute delivery tasks using a SMACH-based state machine
- Support both handcrafted and scanning-based positioning

---

## 🧠 System Architecture

Main components:
- **Navigation**: ROS move_base and 2D Nav Goal
- **Perception**: YOLO-based person detection
- **Coordinate Transformation**: Camera → map frame
- **Decision Making**: SMACH state machine
- **Database**: YAML-based position storage

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

Install dependencies:

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

## 🔁 Reproducibility / Setup Experiment

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

### 5. Optional

```bash
rosrun ta2_farhan sm_new.py
```

---

## 🔍 Features

- Real-time person detection using YOLO
- Hybrid navigation (handcrafted + scanning mode)
- Coordinate transformation (camera → map)
- SMACH-based modular state machine
- Database-driven position system

---

## 📊 Experiment

Evaluated with:
- Multiple delivery scenarios
- Static and dynamic person positions

Metrics:
- Navigation accuracy (x, y error)
- Detection success rate

---

## 📌 Notes

- Replace `<your_workspace>` with your own path (e.g., `/home/user`)
- Ensure ROS environment is sourced before running
- Camera topic must be active
- Map must be loaded before starting navigation

---

## 👤 Author

**Farhan Firmansyah**  
TA2 Delivery Robot Project
