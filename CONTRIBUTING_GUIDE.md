# 🤝 Contributing Guide – McMaster Drone Club Autonomy Repo

This document explains how team members contribute, push changes, open PRs, and maintain consistent code quality.

---

# 🧭 Branching Strategy

- **main** – stable production branch  
- **feature/<name>** – new features  
- **fix/<name>** – bug fixes  
- **experiment/<name>** – prototypes  

Example:
```
git checkout -b feature/aruco-improvement
```

---

# 🧪 Development Workflow

## 1️⃣ Create a branch  
```
git checkout -b feature/<feature-name>
```

## 2️⃣ Write code  
Modify:
- ROS nodes  
- Python scripts  
- Gazebo assets  
- Mission logic  
- ML models  

## 3️⃣ Test thoroughly  
Before pushing:

### ROS:
```
cd courseRoot/catkin_ws
catkin_make
```

### Gazebo:
```
roslaunch gazebo_ros iris_world.launch
```

### SITL:
```
cd apm/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.py -f gazebo-iris
```

### Precision landing:
```
rosrun gazebo_drone gazebo_precision_landing.py
```

### DroneKit:
```
python3 dk/drone/<script>.py
```

---

# 📤 Pushing Changes

```
git add .
git commit -m "Describe your change"
git push -u origin feature/<branch>"
```

---

# 🔍 Opening a Pull Request

Every PR must include:

### ✔ Summary of changes  
### ✔ Before/After results  
### ✔ Testing procedure  
### ✔ Affected systems:  
- ROS  
- Gazebo  
- DroneKit  
- ML  
- Models/worlds  

### ✔ Screenshots/gifs (if applicable)  
### ✔ Checklist:  
- [ ] Builds  
- [ ] Tested in simulation  
- [ ] No breaking topics  
- [ ] No world/model breakage  

### Tag reviewers:  
- Team lead  
- Relevant subsystem owners  

---

# 🛑 Direct commits to `main` are forbidden  
All changes MUST come through PRs.

---

# 🧹 Code Style

### Python  
- snake_case  
- docstrings  
- no unused imports  
- group ROS publishers/subscribers  
- keep mission logic modular  

### ROS  
- All new nodes go under `gazebo_drone/scripts`  
- Update `CMakeLists.txt` when needed  
- Document parameters  

### Simulation  
- Place models under `ardupilot_gazebo/models`  
- World files under `ardupilot_gazebo/worlds`  

---

# 🐞 Creating Issues  
Create a GitHub issue when:

- a bug is identified  
- reproducible errors occur  
- improvements are needed  
- documentation gaps exist  
- new features are requested  

Include:
- log output  
- screenshots  
- replication steps  
- affected components  

---

# ✍️ Add Yourself as a Contributor  
In:
```
README.md → Contributors
```

Submit it as your **first PR** to confirm your setup works.

---

# 🏁 End of Contributing Guide
