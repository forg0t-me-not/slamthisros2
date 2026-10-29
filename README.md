# slamthisros2

ROS 2 Humble/Gazebo implementation of a three-stage TurtleBot3 stadium task:

- Task A follows the yellow line, reads the `STOP` QR code, and turns clockwise.
- Task B drives to the staging pose for a selected ArUco ID using Nav2.
- Task C estimates the marker pose and docks to a 10 cm LiDAR clearance.

The complete A → B → C workflow was validated in a clean Ubuntu 22.04 / ROS 2 Humble container with ArUco ID 5. The final dock completed at 10.3 cm clearance, 1.2 cm pose error, 0.1 cm lateral error, and 0.6° heading error.

## Requirements

- Ubuntu 22.04 Jammy
- ROS 2 Humble Desktop
- Gazebo Classic
- TurtleBot3 Gazebo and Navigation2
- OpenCV with ArUco support
- pyzbar/ZBar

Follow the official ROS 2 Humble installation instructions first. They configure the ROS APT repository; Ubuntu's `universe` repository must also be enabled for `python3-pyzbar`.

## Build

Clone the repository into a ROS 2 workspace:

```bash
mkdir -p ~/ros2_humble_ws/src
cd ~/ros2_humble_ws/src
git clone https://github.com/forg0t-me-not/slamthisros2.git
```

Install declared dependencies and build:

```bash
cd ~/ros2_humble_ws
source /opt/ros/humble/setup.bash

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro humble \
  --skip-keys ament_python \
  -r -y

colcon build --symlink-install --packages-select slamthisros2
source install/setup.bash
```

`ament_python` is skipped only during `rosdep install`; keep it as the build type in `package.xml`.

Verify the installation:

```bash
ros2 pkg prefix slamthisros2
ros2 pkg executables slamthisros2
```

## Run

Use a separate terminal for each process. Source both Humble and the workspace in every terminal:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_humble_ws/install/setup.bash
```

### 1. Stadium simulation

```bash
ros2 launch slamthisros2 course_final.launch.py
```

### 2. Nav2

The navigation parameters use the TurtleBot3 Burger profile:

```bash
export TURTLEBOT3_MODEL=burger

MAP_FILE="$(ros2 pkg prefix slamthisros2)/share/slamthisros2/maps/final_map.yaml"

ros2 launch turtlebot3_navigation2 navigation2.launch.py \
  use_sim_time:=True \
  map:="$MAP_FILE"
```

Set the initial robot pose with **2D Pose Estimate** in RViz, then verify that Nav2 is ready:

```bash
ros2 action list | grep /navigate_to_pose
```

### 3. Complete A → B → C workflow

Choose a marker ID from 1 through 5:

```bash
ros2 run slamthisros2 main --marker-id 5
```

The default enables Task C docking motion. Stop the controller with `Ctrl+C`; cleanup publishes a zero velocity command.

## Safer Task C checks

Measure a Task C docking goal without moving:

```bash
ros2 run slamthisros2 main --marker-id 5 --skip-ab --dry-c
```

Run Task C from the current robot pose with docking enabled:

```bash
ros2 run slamthisros2 main --marker-id 5 --skip-ab --move-c
```

Use a different LaserScan topic if necessary:

```bash
ros2 run slamthisros2 main --marker-id 5 --scan-topic /scan
```

## ROS interfaces

| Interface | Purpose |
| --- | --- |
| `/camera/image_raw` | Tasks A and C vision |
| `/camera/camera_info` | Task C camera intrinsics |
| `/cmd_vel` | Robot velocity commands |
| `/odom` | Task C odometry feedback |
| `/scan` | Task C final-clearance safety |
| `/navigate_to_pose` | Task B Nav2 action |

Task A displays an OpenCV camera window and therefore requires a graphical desktop session.

