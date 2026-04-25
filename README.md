# TA2 Delivery Robot 🚀

This repository presents a ROS-based autonomous delivery robot that integrates YOLO for person detection and SMACH for task execution. The system enables a robot to navigate to a target location, detect a person, and perform delivery tasks autonomously.

---

## 📌 Overview

The system is designed to:
- Perform autonomous navigation using ROS Navigation Stack
- Detect and localize a person using YOLO
- Execute delivery tasks using a SMACH-based state machine
- Support both handcrafted and scanning-based positioning

---

## 🧠 System Architecture

The main components include:
- **Navigation**: ROS move_base and 2D Nav Goal
- **Perception**: YOLO-based person detection
- **Coordinate Transformation**: Convert detection to map frame
- **Decision Making**: SMACH state machine
- **Database**: YAML-based position storage

---

## 📂 Repository Structure


ta2_farhan/
├── config/ # Position database (YAML)
├── launch/ # ROS launch files
├── maps/ # Map files for navigation
├── scripts/ # Main Python scripts
│ ├── sm.py
│ ├── sm_new.py
│ ├── smm.py
│ ├── yolo_hybrid.py
│ ├── coordinate_transformer.py
│ ├── scanning_mode.py
│ ├── db_visualizer.py
│ ├── live_person_marker.py
│ └── ...
├── CMakeLists.txt
├── package.xml


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
▶️ How to Run
cd <your_workspace>/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ta2_farhan run.launch
🔁 Reproducibility / Setup Experiment

Follow these steps to reproduce the system:

1. Create ROS Workspace
mkdir -p <your_workspace>/catkin_ws/src
cd <your_workspace>/catkin_ws
catkin_make
2. Clone Repository
cd <your_workspace>/catkin_ws/src
git clone https://github.com/farhannnf/delivery-robot-yolo-smach.git
3. Build Workspace
cd <your_workspace>/catkin_ws
catkin_make
source devel/setup.bash
4. Run the System
roslaunch ta2_farhan run.launch
5. (Optional) Run State Machine
rosrun ta2_farhan sm_new.py
🔍 Features
Real-time person detection using YOLO
Hybrid navigation (handcrafted + scanning mode)
Coordinate transformation from camera to map
Modular state machine using SMACH
Database-driven position handling
📊 Experiment

The system was evaluated using:

Multiple delivery scenarios
Static and dynamic person positions

Metrics:

Navigation accuracy (x, y error)
Detection success rate
📌 Notes
Replace <your_workspace> with your own directory (e.g., /home/user)
Ensure ROS environment is sourced before running
Camera topic must be active for detection
Map must be loaded before navigation
📜 License

This project is intended for academic purposes.

👤 Author

Farhan Firmansyah
TA2 Delivery Robot Project
