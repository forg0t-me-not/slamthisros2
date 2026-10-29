# slamthisros2
- **Task A:** Follow the yellow line, detect the `STOP` QR code, stop, and turn 90° clockwise.
- **Task B:** Navigate with Nav2 to the staging pose selected by ArUco ID.
- **Task C:** Align with the ArUco marker and dock at 10 cm LiDAR clearance.

> Simulation only. Tested with Ubuntu 22.04, ROS 2 Humble, Gazebo Classic, and TurtleBot3 Burger.

## Install and build

```bash
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y python3-rosdep python3-colcon-common-extensions
```

Clone the repository:

```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws/src
git clone https://github.com/forg0t-me-not/slamthisros2.git
```

Install dependencies and build:

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

rosdep update
rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  --skip-keys ament_python \
  -r -y

colcon build --symlink-install --packages-select slamthisros2
source install/setup.bash
```

## Run

Use three terminals. In each terminal, run:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
```

### 1. Start Gazebo

```bash
ros2 launch slamthisros2 course_final.launch.py
```

### 2. Start Nav2

```bash
export TURTLEBOT3_MODEL=burger
MAP_FILE="$(ros2 pkg prefix slamthisros2)/share/slamthisros2/maps/final_map.yaml"

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:="$MAP_FILE"
```

Set the initial robot pose using **2D Pose Estimate** in RViz and click on the robot. Confirm that Nav2 is ready:

```bash
ros2 action list | grep /navigate_to_pose
```

### 3. Run the complete A → B → C workflow

Choose a marker ID from 1 to 5:

```bash
ros2 run slamthisros2 main --marker-id 5
```

Stop the controller with `Ctrl+C`.
