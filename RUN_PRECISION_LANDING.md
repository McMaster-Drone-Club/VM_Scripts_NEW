# 🛬 Precision Landing – Full Run Guide (DroneDojo VM)

This document walks through the exact steps needed to run the **DroneDojo Precision Landing Course** inside the McMaster Drone Club autonomy VM.

---

# 🟦 STEP 1 — Launch Gazebo (Simulation Environment)

Open a **new terminal**:

```
cd ~
cd courseRoot/catkin_ws/
catkin_make
chmod +x src/gazebo_drone/scripts/*
roslaunch gazebo_ros iris_world.launch
```

### 🔧 Inside Gazebo UI
Go to:

**Scene → Disable Shadows**

(required for proper ArUco detection)

---

# 🟩 STEP 2 — Launch SITL (Simulated Drone)

Open a **new terminal**:

```
cd ~
cd courseRoot/apm/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris
```

### ⚠️ If this fails:
Try again:
```
../Tools/autotest/sim_vehicle.py -f gazebo-iris
```

SITL may require 1–3 attempts based on VM load.

---

# 🟨 STEP 3 — Ensure GPS Activation
WAIT until SITL prints:

```
GPS has been activated
```

This may take **up to 3 minutes** and is **required**.

This solves:
- MAVROS heartbeat errors  
- ROS connection failures  
- ArUco timing issues  

---

# 🟥 STEP 4 — View the Downward Camera via RQT

Open a **new terminal**:

```
rqt
```

In RQT:
- Plugins → Visualization → Image View  
- Choose `/gazebo_drone/camera/image_raw`  

---

# 🟦 STEP 5 — Run the Precision Landing Node

Open a **new terminal**:

```
rosrun gazebo_drone gazebo_precision_landing.py
```

This node performs:
- camera subscription  
- ArUco detection  
- precision landing guidance  
- descent control state machine  

---

# 🟪 Optional: Precision Landing Setup Script
If enabled:

```
./precision_landing_setup.sh
```

But if SITL fails, re-run:

```
../Tools/autotest/sim_vehicle.py -f gazebo-iris
```

---

# 🛠 Troubleshooting

### ❌ SITL won’t start  
→ Try launching again.

### ❌ Gazebo shows black camera  
→ Disable shadows.

### ❌ No camera in RQT  
→ Confirm ROS topics:
```
rostopic list
```

### ❌ Precision landing node fails  
→ Ensure GPS activation step completed.

---

# 🏁 End of Precision Landing Guide
