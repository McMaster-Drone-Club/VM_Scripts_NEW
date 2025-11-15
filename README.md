# VM_Scripts_NEW

# 🛸 McMaster Drone Club – DroneDojo Simulation / Autonomy Workspace

Unified repository for **simulation**, **precision landing**, **DroneKit autonomy**, **Gazebo worlds**, and **ROS perception** used for internal deployment and AEAC competition.

This repo is extracted from the **DroneDojo VM courseRoot** and contains the editable code needed for our drone autonomy stack:

- **ROS (gazebo_drone)**: Camera pipeline, ML, ArUco detection, precision landing
- **Gazebo / ArduPilot-Gazebo**: Models, worlds, simulations
- **DroneKit (dk)**: Mission logic, velocity commands, autonomous navigation
- **ML integration**
- **Team workflow, documentation, and onboarding**

---

# 📁 Repository Structure

```
courseRoot/
│
├── ardupilot_gazebo/            # Worlds, models, scripts for SITL/Gazebo integration
│
├── catkin_ws/
│   ├── src/
│   │   ├── gazebo_drone/        # Main ROS autonomy package
│   │   └── other ROS packages   # rqt_image_view, ros_numpy, etc.
│   └── build / devel / logs     # Auto-generated (ignored)
│
├── dk/                          # DroneKit mission scripts
├── ml/                          # Optional ML models (vision, detectors)
└── README.md
```

---

# 🧰 Developer Setup (Onboarding)

## 1️⃣ Clone this repository inside the DroneDojo VM
```
cd ~/courseRoot
git init
git remote add origin https://github.com/McMaster-Drone-Club/VM_Scripts_NEW
git pull origin main --allow-unrelated-histories
```

## 2️⃣ Remove nested Git repos that came with the DroneDojo VM
```
rm -rf ardupilot_gazebo/.git
rm -rf catkin_ws/src/ros_numpy/.git
rm -rf catkin_ws/src/rqt_image_view/.git
rm -rf .gitmodules
```

## 3️⃣ Configure your Git identity
```
git config --global user.name "YOUR NAME"
git config --global user.email "you@example.com"
```

## 4️⃣ Authenticate with GitHub
Use **SSH keys** (recommended) or **Personal Access Tokens**.

---

# 🧪 Running the Precision Landing Simulation
Full step-by-step instructions are available in:
➡ **RUN_PRECISION_LANDING.md**

---

# 🛠 Running the Full Simulation Pipeline

## 1. Build ROS
```
cd ~/courseRoot/catkin_ws
catkin_make
source devel/setup.bash
```

## 2. Launch Gazebo
```
roslaunch gazebo_ros iris_world.launch
```

## 3. Launch SITL
```
cd ~/courseRoot/apm/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris
```

## 4. Start the precision landing node
```
rosrun gazebo_drone gazebo_precision_landing.py
```

## 5. Run a DroneKit mission
```
python3 dk/drone/taco_delivery.py
```

---

# 🔄 Updating Your VM When Teammates Commit Changes

```
cd ~/courseRoot
git pull origin main
```

Then rebuild ROS:
```
cd catkin_ws
catkin_make
```

If worlds/models change:
```
rm -rf ~/.gazebo/models/*
```

If Python requirements change:
```
pip install -r requirements.txt
```

---

# 📚 File Purpose Summary

### ✔ ardupilot_gazebo/
Gazebo models, worlds, and plugin scripts used with ArduPilot.

### ✔ gazebo_drone/
ROS package with:
- Camera subscriber
- ML/ArUco detection
- Precision landing controller
- MAVROS/ROS integration

### ✔ dk/
DroneKit Python autonomy scripts.

### ✔ ml/
Machine learning model storage (future expansion).

---

# 🤝 Contributing

See **CONTRIBUTING.md** for:
- PR workflow
- Branching conventions
- Testing requirements
- Code review process

---

# ✍️ Contributors
(Add your name in your first PR.)

```
- Sachin Gupta — Autonomy / ML / DroneKit
- <Your Name> — <Your Role>
```

---

# 🏁 End of README
