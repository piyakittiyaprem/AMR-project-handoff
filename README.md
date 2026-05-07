# AMR ROS Noetic Workspace

This repository contains the ROS Noetic workspace for the AMR senior project.

## Workspace packages

- `amr_bringup`: core runtime bringup launch files.
- `amr_description`: robot model and TF/display launch files.
- `amr_sensors`: LiDAR and IMU launch integration.
- `amr_teensy_bridge`: ROS <-> Teensy serial bridge.
- `amr_manual_teleop`: keyboard teleoperation node and launch.
- `amr_autonomous_mapping`: SLAM support nodes and SLAM launch.
- `amr_navigation`: navigation stack config and RViz profiles.

## Environment setup 

### 1) Copy project files to Jetson Nano workspace

From laptop/host machine, copy this repository into Jetson catkin workspace `src`:

```bash
scp -r "<local_project_path>" <jetson_user>@<jetson_ip>:~/catkin_ws/src/
```

Then on Jetson:

```bash
cd ~/catkin_ws/src
ls
```

Pass criteria:
- Project folder appears under `~/catkin_ws/src`.

### 2) Build and source on Jetson

From Jetson workspace root:

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

Pass criteria:
- `catkin_make` completes with no build errors.
- `source devel/setup.bash` returns with no errors.

Pass criteria:
- `catkin_make` completes with no build errors.
- `source devel/setup.bash` returns with no errors.

## Start ROS master

Use a dedicated terminal:

```bash
roscore
```

Pass criteria:
- Terminal prints `started core service [/rosout]`.

## Runtime modes

### Mode 1: Manual Teleop

Launch sequence:

```bash
# Terminal A
roslaunch amr_bringup bringup_base.launch
```

```bash
# Terminal B
roslaunch amr_manual_teleop manual_teleop_wasd.launch
```

Safety step before floor testing:
- Lift the robot off the ground first and check wheel direction/kinematics.

Verification commands:

```bash
rosnode list
rostopic list
rostopic hz /cmd_vel
rostopic hz /odom
rostopic hz /scan
rostopic echo -n 1 /cmd_vel
```

Pass criteria:
- `rosnode list` contains `teensy_bridge` and `manual_teleop_wasd`.
- `/cmd_vel`, `/odom`, and `/scan` appear in `rostopic list`.
- `/odom` and `/scan` show steady non-zero publish rate.
- Pressing teleop keys changes `/cmd_vel` values.

### Mode 2: Autonomous SLAM

Launch sequence:

```bash
roslaunch amr_autonomous_mapping slam_autonomous.launch
```

Verification commands:

```bash
rosnode list
rostopic list
rostopic hz /scan
rostopic hz /odom
rostopic echo -n 1 /slam_out_pose
rosrun tf view_frames
```

Pass criteria:
- `rosnode list` contains `hector_mapping` and `slam_pose_to_odom`.
- `/scan` and `/odom` publish at steady non-zero rate.
- `/slam_out_pose` returns valid pose output.
- TF graph is generated with required map/odom/base links.

## Documentation

- Full troubleshooting guide: `docs/TROUBLESHOOTING_GUIDELINES.md`.
- Wiring details: `docs/wiring config.txt`.

## Contact

- Maintainer contact: `piyakittiyaprem@gmail.com`
