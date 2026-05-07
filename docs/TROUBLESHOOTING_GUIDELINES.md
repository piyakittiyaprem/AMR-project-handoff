# AMR Troubleshooting Guidelines (Software + Hardware)

This document is the troubleshooting guide for operation.

## 1) Pre-Power Safety Checklist

1. Measure LiPo battery voltage before operation.
   - Do not run the robot below `21.5 V`.
2. Measure LiDAR supply voltage using a multimeter.
   - Acceptable range: `5.0 V` to `5.2 V`.
3. Confirm all connectors are fully seated and aligned correctly.
4. Confirm wire pins are tight and not slipping out of connectors.
5. Check encoder and motor driver connectors for looseness.
   - When each wheel is rotated by hand, the matching encoder must rotate too.

## 2) Hardware LED Indicators

- **Jetson Nano:** yellow LED ON.
- **LiDAR:** green LED ON.
- **CP2102 USB-UART adapter:** red LEDs ON.
- **MPU6050:** yellow LED ON.

If any expected LED is OFF, fix power and wiring first before software debugging.

## 3) Wiring Rules

1. Cross UART lines correctly: `RX -> TX` and `TX -> RX`.
2. Verify connector orientation using the included wiring photo.
3. Use `docs/wiring config.txt` for detailed pin mapping.

## 4) Core Software Bringup Checks

From workspace root:

```bash
catkin_make
source devel/setup.bash
roscore
```

Then run:

```bash
rosnode list
rostopic list
ls /dev/ttyUSB0 /dev/ttyACM0
rostopic hz /scan
rostopic hz /odom
rostopic hz /imu/data
rostopic echo -n 1 /scan
rostopic echo -n 1 /odom
rostopic echo -n 1 /imu/data
rosrun tf view_frames
```

Pass criteria:

- `catkin_make` succeeds with no errors.
- `ls /dev/ttyUSB0 /dev/ttyACM0` returns at least one valid serial device path.
- `/scan`, `/odom`, and `/imu/data` exist in `rostopic list`.
- `/scan` and `/odom` have steady non-zero rates.
- `rostopic echo -n 1` returns valid data for all required topics.
- TF graph is generated and includes expected map/odom/base links.

If `/scan` is missing:

```bash
ls /dev/ttyUSB0 /dev/ttyACM0
rostopic list | grep scan
rosnode list | grep -E "rplidar|ydlidar"
```

Then re-check:
- LiDAR LED status
- LiDAR supply voltage (`5.0-5.2 V`)
- LiDAR port and baud in launch files

## 5) Mode-Specific Operation and Troubleshooting

### Mode 1: Manual Teleop

1. Start ROS master:
   ```bash
   roscore
   ```
2. Launch base stack:
   ```bash
   roslaunch amr_bringup bringup_base.launch
   ```
3. Launch teleop:
   ```bash
   roslaunch amr_manual_teleop manual_teleop_wasd.launch
   ```
4. Verify:
   ```bash
   rosnode list
   rostopic list
   rostopic hz /cmd_vel
   rostopic hz /odom
   rostopic hz /scan
   rostopic echo -n 1 /cmd_vel
   rostopic echo -n 1 /odom
   ```

Pass criteria:

- `rosnode list` includes `teensy_bridge` and `manual_teleop_wasd`.
- `/cmd_vel`, `/odom`, and `/scan` appear in `rostopic list`.
- `/odom` and `/scan` have steady non-zero rates.
- Keyboard input changes `/cmd_vel` values.

Safety note:
- Lift the robot off the ground first to verify wheel kinematics before floor tests.

If motion is incorrect:

```bash
rostopic hz /cmd_vel
rostopic echo -n 1 /cmd_vel
rosnode list | grep teensy_bridge
ls /dev/ttyUSB0 /dev/ttyACM0
```

Then inspect relay direction and encoder mapping in `docs/wiring config.txt`.

### Mode 2: Autonomous SLAM

1. Start ROS master:
   ```bash
   roscore
   ```
2. Launch:
   ```bash
   roslaunch amr_autonomous_mapping slam_autonomous.launch
   ```
3. Verify:
   ```bash
   rosnode list
   rostopic list
   rostopic hz /scan
   rostopic hz /odom
   rostopic echo -n 1 /slam_out_pose
   rosrun tf view_frames
   ```

Pass criteria:

- `rosnode list` includes `hector_mapping` and `slam_pose_to_odom`.
- `/scan` and `/odom` publish at steady non-zero rate.
- `/slam_out_pose` returns valid pose output.
- TF graph includes required map/odom/base links.

If SLAM is unstable:

```bash
rostopic hz /scan
rostopic hz /imu/data
rostopic echo -n 1 /imu/data
rosrun tf view_frames
```

## 6) Viewing Jetson Desktop (Access Methods)

Practical workflow from my experience (no dedicated monitor screen, but only laptop, dummy HDMI ,and micro-usb):
1. Use **Method D** once to get Jetson IP.
2. Connect via **Method E (SSH)** for terminal operations.
3. Use **Method B (NoMachine)** for remote desktop.

### Method A: Direct monitor + keyboard + mouse

- Connect Jetson to monitor (HDMI/DisplayPort depending on hardware).
- Connect keyboard and mouse.
- If no monitor is available, use HDMI + capture card to view desktop on laptop.
- Best for first bringup and lowest-latency debugging.

### Method B: NoMachine (wireless remote desktop)

- NoMachine is installed on Jetson Nano (March 2026 setup).
- Plug dummy HDMI into Jetson if required to force desktop rendering.
- Install NoMachine client on laptop.
- Ensure Jetson and laptop are on the same local network.
- Connect to Jetson IP from NoMachine client.

### Method C: VNC (alternative GUI)

- Use VNC when NoMachine is unavailable.
- Enable VNC server on Jetson and connect via VNC client.

### Method D: USB-C access to obtain Jetson IP (if supported)

- Connect USB-C between Jetson and host PC.
- Open PuTTY and connect over serial (COM port, `115200` baud).
- In Jetson terminal, run:
  ```bash
  hostname -I
  ip a
  ```
- Identify active Jetson IP, then use it for SSH/NoMachine/VNC.

### Method E: SSH (terminal access)

From Windows PowerShell:

```bash
ssh <jetson_user>@<jetson_ip>
```

From PuTTY:
- Host Name: `<jetson_ip>`
- Port: `22`
- Connection Type: `SSH`

After login, verify:

```bash
whoami
hostname
```

## 7) LiPo Charging Procedure (SkyRC B6 Neo)

1. Select battery type: `LiPo`.
2. Select cell count: `6S`.
3. Connect main lead (`XT60`).
4. Connect balance port.
5. Start charging with safe current for pack specification.
6. Monitor for abnormal heat/swelling during charge.
7. Full charge around `25.0 V` to `26.0 V` is normal for a 6S pack, despite being rated at `22.2 V`.

## 8) Powering the NVIDIA Jetson Nano

To ensure stable performance, especially when running intensive AI workloads or ROS nodes, selecting the right power source is critical. Below are the three primary methods to power your Jetson Nano:

1. 5V 4A DC Barrel Jack (Recommended)

This is the most reliable method for stationary development and Maximum Power Mode (10W).

Setup: Requires a jumper cap on the J48 pins to enable power through the barrel jack.

Best For: Working on the Jetson independently from the robot or performing heavy computations where high current draw is expected.

Advantage: Prevents "brownouts" (sudden shutdowns) that often occur under heavy load.

2. Micro-USB Port

A convenient but limited power option.

Setup: Ensure the J48 jumper cap is removed to use this port.

Best For: Basic setup, lightweight coding, or when a high-current DC supply isn't available.

Disadvantage: Most Micro-USB cables and chargers cannot consistently deliver the amperage required for the Nano's high-performance modes, which may lead to system instability.

3. Mini560 DC-DC Step-Down (Robot Integration)

The ideal solution for mobile, wireless operation.

Setup: Use a Mini560 (5A) buck converter to regulated your robot's main battery voltage down to a steady 5V, then feed it into the barrel jack.

Best For: Autonomous Mobile Robots (AMR) and field testing.

Advantage: The 5A capacity provides a safety margin above the Nano's 4A requirement, ensuring the Jetson stays powered even when the robot's motors create voltage fluctuations.

## 9) Shutdown Procedure

1. Stop all ROS nodes and teleop commands (`Ctrl+C` per terminal).
2. Turn OFF main power switch.
3. Unplug battery after power-off to avoid parasitic drain and deep discharge.

## 10) Every 30 Minutes During Operation

- Re-check LiPo voltage (must stay above `21.5 V`).
- Re-check connector firmness and cable strain.
- Confirm LiDAR and Jetson LEDs remain normal.
- Listen for unusual motor/encoder noise.

