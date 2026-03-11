
# ROS2 LQRi Trajectory Controller for Quadrotor

This repository contains a **ROS 2 Humble Python controller node** for a quadrotor UAV.
The controller implements:

- **LQR with integral action (LQRi)** for attitude stabilization
- **PID position control** for X-Y motion
- **PID altitude control**
- **Trajectory generation (2D & 3D)**
- **Motor mixing for quadrotor thrust allocation**

The system supports multiple flight modes including hover, straight motion, sine trajectories, helix motion, and figure‑8 trajectories.

---

# Node Overview

### Node Name
```
lqri_trajectory_controller
```

### Subscribed Topics

| Topic | Message | Description |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | Position estimation |
| `/imu` | `sensor_msgs/Imu` | Orientation and angular velocity |
| `/set_target_xyz` | `geometry_msgs/Vector3` | Target position command |
| `/set_flight_mode` | `std_msgs/String` | Flight mode command |

---

### Published Topics

| Topic | Message | Description |
|------|------|------|
| `/motor_commands` | `actuator_msgs/Actuators` | Motor velocity commands |
| `/debug/current_xyz` | `geometry_msgs/Vector3` | Current position |
| `/debug/target_xyz` | `geometry_msgs/Vector3` | Target position |
| `/debug/current_rpy` | `geometry_msgs/Vector3` | Current attitude |
| `/debug/target_rpy` | `geometry_msgs/Vector3` | Target attitude |

---

# Control Architecture

The control system uses a **nested control structure**.

```
Trajectory Generator
        ↓
Position Controller (PID)
        ↓
Attitude Controller (LQRi)
        ↓
Motor Mixing
        ↓
Motor Angular Velocity Commands
```

---

# State Variables

The controller estimates:

Position:
```
x, y, z
```

Orientation:
```
roll, pitch, yaw
```

Angular velocity:
```
p, q, r
```

Data comes from:

- `/odom`
- `/imu`

---

# Attitude Controller (LQRi)

The inner loop uses **LQR with integral action**.

Augmented state:

```
x_aug =
[
∫e_roll
∫e_pitch
∫e_yaw
e_roll
e_pitch
e_yaw
p
q
r
]
```

Control input:

```
u = [τx τy τz]
```

Computed using:

```
u = -K x_aug
```

The gain matrix is computed using:

```
P = solve_continuous_are(A, B, Q, R)
K = R⁻¹ Bᵀ P
```

---

# Position Controller

The outer loop converts **position error → attitude commands**.

### X axis

```
error_x = target_x - curr_x
```

Pitch command:

```
pitch_cmd =
kp_pos * error_x +
ki_pos * integral_x +
kd_pos * derivative_x
```

---

### Y axis

```
error_y = target_y - curr_y
```

Roll command:

```
roll_cmd =
-(kp_pos * error_y +
  ki_pos * integral_y +
  kd_pos * derivative_y)
```

---

# Altitude Controller

Altitude uses PID control.

```
error_z = target_z - curr_z
```

Total thrust:

```
T = mg + PID_z
```

Where:

```
PID_z =
kp_z * error_z +
ki_z * integral_z +
kd_z * derivative_z
```

---

# Motor Mixing

Quadrotor force vector:

```
F = [T τx τy τz]
```

Motor thrusts:

```
T_motor = M⁻¹ F
```

Motor speed:

```
ω = sqrt(T / kF)
```

Limited by:

```
ω ≤ ω_max
```

---

# Flight Modes

| Mode | Description |
|-----|-----|
| `IDLE` | Motors OFF |
| `2D` | Motion in X‑Z plane |
| `3D` | Full XYZ control |

---

# Trajectory Modes

### Hover
Maintain position.

---

### GOTO
Move to user‑defined target.

---

### Straight Motion

Forward:

```
x = x0 + 3t
```

Backward:

```
x = x0 - 3t
```

---

### Sine (2D)

```
x = x0 + 0.3t
z = z0 + sin(0.5t)
```

---

### Ramp (2D)

```
z = z0 + 1.5 * triangle_wave(t)
x = x0 + 0.8t
```

---

### Straight (3D)

```
x = x0 + vx t
y = y0 + vy t
z = z0 + vz t
```

---

### Helix (3D)

```
x = r sin(ωt)
y = r (1 − cos(ωt))
z = z0 + vt
```

Produces a spiral trajectory.

---

### Figure‑8 (3D)

```
x = x0 + (a cos(ωt))/(1 + sin²(ωt)) - a
y = y0 + (a sin(ωt)cos(ωt))/(1 + sin²(ωt))
z = z0 + sin(2ωt)
```

---

# How to Run

The controller is designed to run together with the **quadrotor
simulator** from the `quad_description` package.

The simulator provides:

-   `/odom`
-   `/imu`

and receives motor commands from:

-   `/motor_commands`

### 1) Clone
```bash
git clone -b Lab2 https://github.com/Pungpond3947/Mobile_Robot.git
```

### 2) Build
```bash
cd Mobile_Robot
colcon build
source install/setup.bash
```
------------------------------------------------------------------------

## Launch the Simulation

Open **Terminal 1** and start the quadrotor simulation.

``` bash
ros2 launch quad_description sim.launch.py
```
------------------------------------------------------------------------

## Select Flight Mode

Open **Terminal 3** to control the drone behaviour.

Example:

``` bash
ros2 topic pub /set_flight_mode std_msgs/String "{data: '3D'}"
```

This command:

-   Activates the motors
-   Enables the controller

------------------------------------------------------------------------

# 🧭 Available Flight Modes

  Mode              Description
  ----------------- ----------------------------------------
  `IDLE`            Motors OFF
  `2D`              Motion control in **X-Z plane**
  `3D`              Motion control in **X-Y-Z space**
  `2D_SINE`         Drone follows **sine wave trajectory**
  `2D_STRAIGHT_F`   Move forward along **X axis**
  `2D_STRAIGHT_B`   Move backward along **X axis**
  `2D_RAMP_WAVE`    Triangle wave altitude motion
  `3D_HELIX`        Spiral (helix) trajectory
  `3D_STRAIGHT`     Straight motion in 3D
  `3D_FIGURE8`      Figure‑8 trajectory

Example:

``` bash
ros2 topic pub /set_flight_mode std_msgs/String "{data: '3D_HELIX'}"
```

This will make the drone follow a **3D spiral trajectory**.

------------------------------------------------------------------------

## 4️⃣ Send Target Position (Manual Control)

When the drone is in **2D** or **3D** mode you can send a target
position.

Example:

``` bash
ros2 topic pub /set_target_xyz geometry_msgs/Vector3 "{x: 2.0, y: 1.0, z: 2.0}"
```

Meaning:

    Target position
    x = 2.0 m
    y = 1.0 m
    z = 2.0 m

The controller will automatically convert:

    position error → attitude command → motor command

------------------------------------------------------------------------

## 5️⃣ Stop the Drone

To stop the drone:

``` bash
ros2 topic pub /set_flight_mode std_msgs/String "{data: 'IDLE'}"
```

This command:

-   Turns motors OFF
-   Stops the controller

------------------------------------------------------------------------

# Control Frequency

```
100 Hz
```

Timer period:

```
0.01 s
```

---

# 🔧 Tuning Parameters

Position controller:

```
kp_pos
ki_pos
kd_pos
```

Altitude controller:

```
kp_z
ki_z
kd_z
```

LQR matrices:

```
Q = state penalty
R = control penalty
```

---

# Suggested Repository Structure

```
quadrotor_control/
│
├── quadrotor_control/
│   └── lqri_trajectory_controller.py
│
├── launch/
│   └── controller.launch.py
│
├── config/
│   └── controller_params.yaml
│
└── README.md
```

---

# 👨‍💻 Author

Khunanon Sawetkhotchakul 66340500006
Robotics & Automation Engineering  
Institute of Field Robotics (FIBO)  
King Mongkut's University of Technology Thonburi
