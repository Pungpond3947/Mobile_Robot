# 🤖 LAB 1: Mobile Robot SLAM & Multi‑Sensor Fusion (Code Walkthrough)

This document explains the provided ROS 2 Humble Python implementation for:

- **Wheel Odometry**
- **EKF (Wheel + IMU yaw fusion)**
- **LiDAR ICP scan matching (keyframe-based)**
- **TF broadcasting + Path visualization**
- **SLAM monitoring + slam_toolbox configuration**
- **Launch file wiring**

> Designed for **2D Differential Drive** localization on ROS bag data (`/joint_states`, `/imu`, `/scan`).

---

## 📦 Nodes & Topics

### 1) `turtlebot.py` — EKF Core + Wheel Odometry
**Subscriptions**
- `/joint_states` (`sensor_msgs/JointState`) @ ~20 Hz  
- `/imu` (`sensor_msgs/Imu`) @ ~20 Hz

**Publications**
- `/turtle_pose_wheel` (`geometry_msgs/PoseStamped`) — raw wheel integration  
- `/turtle_pose_EKF` (`geometry_msgs/PoseStamped`) — EKF-fused pose (yaw corrected by IMU)

**Frames**
- Published poses use `frame_id: "odom"`.

---

### 2) `icp_ekf.py` — LiDAR ICP Odometry Refinement (Keyframe ICP)
**Subscriptions**
- `/scan` (`sensor_msgs/LaserScan`) @ ~5 Hz *(BEST_EFFORT QoS)*
- `/turtle_pose_EKF` (`geometry_msgs/PoseStamped`) — provides **initial guess** for ICP

**Publications**
- `/turtle_pose_ICP` (`geometry_msgs/PoseStamped`) — refined odometry pose in `odom`

**Core Idea**
- Convert `/scan` ranges into a 2D point set
- Run **point-to-point ICP** aligning current scan to the **latest keyframe scan**
- Use EKF relative motion to build the initial transform guess
- Integrate ICP relative transform into a global odometry pose

---

### 3) `turtlebot_pose.py` — TF Broadcaster + Path Manager
**Subscriptions**
- `/turtle_pose_wheel`
- `/turtle_pose_EKF`
- `/turtle_pose_ICP`

**Publications**
- `/path_wheel` (`nav_msgs/Path`)
- `/path_ekf` (`nav_msgs/Path`)
- `/path_icp` (`nav_msgs/Path`)

**TF**
- Broadcasts transforms (parent: `odom`) to:
  - `base_link_wheel`
  - `base_link_ekf`
  - `base_link_icp`

This lets you visualize each trajectory simultaneously in RViz.

---

### 4) `slam_path.py` — SLAM Pose Monitor (Map-frame Path)
**TF Lookup**
- Looks up transform: `map -> base_link_icp` using TF2

**Publications**
- `/path_slam` (`nav_msgs/Path`) in frame `map`

This is meant to visualize the *optimized* SLAM trajectory compared to raw odometry paths.

---

## 🧠 Part 1 — EKF Odometry Fusion (`turtlebot.py`)

### ✅ State Definition
The EKF state is:
```math
x = [x, y, \theta]^T
```

### ✅ Prediction (Wheel Odometry)
Wheel velocity is computed from `/joint_states.velocity`:
- left wheel: `v_l`
- right wheel: `v_r`

Then:
```text
v     = (v_r + v_l)/2
omega = (v_r - v_l)/L
```

Pose integration (Euler):
```text
theta_k = theta_{k-1} + omega * dt
x_k     = x_{k-1} + v * cos(theta_{k-1}) * dt
y_k     = y_{k-1} + v * sin(theta_{k-1}) * dt
```

> **Note:** Your code intentionally **does NOT multiply by wheel radius `R`** because the joint velocity is assumed already in **m/s**.

### ✅ Jacobian & Covariance Propagation
Motion Jacobian (3×3) is updated using traveled distance `d_k = v*dt`:
```text
F = I
F[0,2] = -d_k * sin(theta_prev)
F[1,2] =  d_k * cos(theta_prev)
```

Then:
```text
P = F P F^T + Q
```

Default values in code:
- `P0 = diag([0.1, 0.1, 0.1])`
- `Q  = diag([0.8, 0.8, 0.8])`

> If the EKF feels “too noisy” or “too confident”, tuning **Q** and **R** is the first place to adjust.

---

### ✅ Correction (IMU Yaw)
IMU orientation quaternion is converted to yaw. The first IMU yaw is stored as `initial_yaw`, then yaw is **zeroed**:
```text
yaw = yaw_imu - initial_yaw
yaw = atan2(sin(yaw), cos(yaw))   # normalize to (-pi, pi]
```

Measurement model uses only yaw:
```text
z = theta
H = [0, 0, 1]
```

Kalman gain:
```text
K = P H^T (H P H^T + R)^-1
```

Update:
```text
error = wrapToPi(yaw - theta_pred)
x = x_pred + K * error
P = (I - K H) P
```

Default measurement noise:
- `R = [0.3]`

---

## 📡 Part 2 — ICP Odometry Refinement (`icp_ekf.py`)

### ✅ Point Cloud Construction
From `LaserScan`, valid points are selected and projected into 2D:
```text
x_i = r_i cos(angle_i)
y_i = r_i sin(angle_i)
```

### ✅ Keyframe ICP (Corridor Stabilization)
Instead of matching scan-to-scan, you match **current scan to a stored keyframe**:
- Helps reduce “corridor shrink” problems
- Stabilizes alignment when geometry is repetitive

#### 1) Initial Guess from EKF
Compute EKF relative motion between current pose and keyframe EKF pose:
```text
dx_world = x_ekf - x_key_ekf
dy_world = y_ekf - y_key_ekf
dth_guess = wrapToPi(theta_ekf - theta_key_ekf)
```

Convert world Δ to keyframe local coordinates:
```text
[dx_guess, dy_guess] = R(-theta_key_ekf) * [dx_world, dy_world]
```

Build initial transform `T_guess` and pre-transform the current scan.

#### 2) ICP Loop (10 iterations)
- Nearest neighbor matching (brute force)
- Reject pairs with distance > **0.2 m**
- Compute best-fit rigid transform via SVD
- Compose incremental transforms: `final_T = T_i @ final_T`

#### 3) Integrate ICP result into global pose
Extract relative transform:
```text
dx_k  = final_T[0,2]
dy_k  = final_T[1,2]
dth_k = atan2(final_T[1,0], final_T[0,0])
```

Update global ICP pose using keyframe ICP pose:
```text
[x,y] = key_icp_xy + R(theta_key_icp) * [dx_k, dy_k]
theta = wrapToPi(theta_key_icp + dth_k)
```

#### 4) Keyframe Update Rule
A new keyframe is created when:
- translation > **0.15 m** OR
- rotation > **0.1 rad** (~5.7°)

---

## 🌍 TF + Paths (`turtlebot_pose.py`)

For each pose topic:
- Broadcast TF: `odom -> base_link_*`
- Append pose into corresponding `nav_msgs/Path`

Paths:
- `/path_wheel`
- `/path_ekf`
- `/path_icp`

This makes RViz comparison straightforward.

---

## 🗺 SLAM Monitoring (`slam_path.py`)

A timer (10 Hz) looks up TF:
```text
map -> base_link_icp
```

Then publishes `/path_slam` in frame `map`.  
This is used to compare **SLAM optimized trajectory** vs. odometry tracks.

---

## 🚀 Launch File Wiring

Your launch file starts:

- `turtlebot.py`
- `turtlebot_pose.py`
- `icp_ekf.py`
- `slam_path.py`
- `rviz2` (loads config)
- `slam_toolbox` (`async_slam_toolbox_node`)
- `static_transform_publisher`

### Static TF
You publish:
```text
base_link_icp -> base_scan
```
with identity transform.

> If your LiDAR is not perfectly aligned at the robot origin, this should be updated to the real extrinsics.

---

## ⚙️ slam_toolbox Parameters (Provided)

```yaml
slam_toolbox:
  ros__parameters:
    odom_frame: odom
    base_frame: base_link_icp
    map_frame: map
    scan_topic: /scan

    mode: mapping
    use_sim_time: true

    do_loop_closing: true
    max_lasso_dist: 2.0
    minimum_time_interval: 0.5

    max_condition_number: 10000.0
    max_res_dist: 0.05
    minimum_angle_penalty: 0.9
    minimum_distance_penalty: 0.5
```

### Notes
- `base_frame` is set to **`base_link_icp`** so SLAM uses your ICP-refined odometry chain.
- `do_loop_closing: true` enables loop closure + global optimization.

---

## ✅ How to Run (Typical)

### 1) Clone
```bash
git clone -b Lab1 https://github.com/Pungpond3947/Mobile_Robot.git
```

### 2) Build
```bash
cd Mobile_Robot
colcon build
source install/setup.bash
```

### 3) Launch
```bash
ros2 launch lab1 turtlebot.launch.py
```

### 4) Play bag (example)
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq00/fibo_floor3_seq00_0.db3 --clock
```
or
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq01/fibo_floor3_seq01_0.db3 --clock
```
or
```bash
ros2 bag play FRA532_LAB1_DATASET/fibo_floor3_seq02/fibo_floor3_seq02_0.db3 --clock
```

### 4) RViz
RViz should display:
- Wheel / EKF / ICP paths in `odom`
- SLAM path in `map`
- TF tree for `base_link_*`

---

## 🔧 Tuning Tips

### EKF
- If EKF yaw follows IMU too aggressively → **increase `R`**
- If EKF is too slow / too “stiff” → **increase `Q`**
- Consider making `Q` anisotropic (e.g., smaller for x/y, larger for theta)

### ICP
- If ICP diverges → lower pairing threshold (0.2 → 0.1) or reduce max iterations
- If ICP is jittery → increase keyframe distance threshold or add downsampling
- Nearest neighbor is O(N²) here; for speed, use KD-tree (optional improvement)

### TF & Frames
- Ensure `odom`, `map`, and `base_scan` are consistent
- Verify SLAM uses the intended base frame (`base_link_icp`)

---

## ⚠️ Known Assumptions / Limitations

- `/joint_states.velocity` is assumed in **m/s**, not rad/s.
- EKF update uses **only yaw** (no position measurement).
- ICP uses brute-force nearest neighbor → may be slow for dense scans.
- Static TF `base_link_icp -> base_scan` is identity; real sensor offset should be used if known.

---

## 📂 Suggested Repo Layout

```text
lab1/
├── lab1/
│   ├── turtlebot.py
│   ├── icp_ekf.py
│   ├── turtlebot_pose.py
│   └── slam_path.py
├── launch/
│   └── main.launch.py
├── slam_config/
│   └── slam_params.yaml
├── rviz2_config/
│   └── rviz2.config.rviz
├── images/
│   ├── system_architecture.png
│   └── robot_dimension.png
└── README.md
```

---

## 👨‍💻 Author

**Kunanon Sawetkotchakul**  
Robotics & Automation Engineering — Institute of Field Robotics (FIBO)
