# 🤖 LAB 1: Mobile Robot SLAM & Multi-Sensor Fusion

> 🔎 A complete study of Localization, Sensor Fusion, and SLAM for a
> 2-Wheel Differential Drive Robot using ROS 2 Humble.

------------------------------------------------------------------------

# 📌 Overview

โปรเจกต์นี้เป็นการพัฒนาระบบ **Localization & Mapping** สำหรับหุ่นยนต์แบบ
Differential Drive โดยเน้นการแก้ปัญหา **Odometry Drift** ผ่านการทำ
**Multi-Sensor Fusion (EKF)** และ **Scan Matching (ICP)**
พร้อมทั้งศึกษาการทำงานของ **Pose Graph SLAM** ด้วย `slam_toolbox`.

## แนวคิดหลักของระบบ

-   ลดความคลาดเคลื่อนจาก Wheel Encoder\
-   ปรับปรุง Pose estimation ด้วย IMU\
-   ใช้ LiDAR Scan Matching ลด error จากล้อ Slip\
-   ทำ Loop Closure เพื่อแก้ error ทั้งระบบแบบ Global Optimization

------------------------------------------------------------------------

# 🎯 Learning Outcomes

## 🔹 1. Sensor Fusion (Extended Kalman Filter)

### State Vector

$$
x = [x, y, \theta]^T
$$

-   ทำ Prediction Step จาก Wheel Odometry\
-   ทำ Correction Step จาก IMU (Gyroscope)\
-   วิเคราะห์ Covariance propagation และ uncertainty reduction

------------------------------------------------------------------------

## 🔹 2. Scan Matching (Iterative Closest Point -- ICP)

-   ใช้ Point-to-Point ICP\
-   ใช้ EKF Pose เป็น Initial Guess\
-   คำนวณ Relative Transform

$$
T_{k-1,k}
$$

-   ลด Error จาก Wheel Slip

------------------------------------------------------------------------

## 🔹 3. Full SLAM System

-   ใช้ `slam_toolbox` (Pose Graph SLAM)\
-   ทำ Loop Closure detection\
-   ทำ Graph Optimization\
-   วิเคราะห์ผลก่อนและหลัง Optimization

------------------------------------------------------------------------

# 🏗 System Architecture

  -----------------------------------------------------------------------
  Node Name                   Role          Description
  --------------------------- ------------- -----------------------------
  `turtlebot.py`              EKF Core      Kinematics +
                                            Prediction/Update ของ EKF

  `icp_ekf.py`                Scan Matcher  ICP Refinement โดยใช้ EKF
                                            Pose

  `turtlebot_pose.py`         TF Manager    Broadcast TF + Publish
                                            `nav_msgs/Path`

  `slam_path.py`              Global        Monitor pose บน `map` frame
                              Monitor       
  -----------------------------------------------------------------------

------------------------------------------------------------------------

## 🧭 ROS 2 Graph Concept

    Wheel Encoder  --->  EKF  --->  ICP Refinement  --->  SLAM Toolbox
            |               |             |                 |
            |               |             |                 |
           IMU ------------>|             |                 |
                                           |                 |
                                       LiDAR -------------->|

------------------------------------------------------------------------

# 📷 System Architecture Diagram

![System Architecture](images/system_architecture.png)

------------------------------------------------------------------------

# 🔬 Methodology

## 🧪 Part 1: EKF Odometry Fusion

$$
x =
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
$$

### Prediction

-   ใช้ Differential Drive Kinematics\
-   Propagate covariance matrix

### Correction

-   ใช้ IMU yaw rate\
-   ลด Drift ใน orientation

------------------------------------------------------------------------

## 🧪 Part 2: ICP Odometry Refinement

1.  รับ Laser Scan ใหม่\
2.  Convert เป็น Point Cloud\
3.  Match กับ Scan ก่อนหน้า\
4.  คำนวณ Transform

$$
T_{k-1,k}
$$

5.  Refine Pose ของ EKF

------------------------------------------------------------------------

## 🧪 Part 3: Full SLAM with `slam_toolbox`

-   Node = Robot pose\
-   Edge = Relative transform\
-   Loop Closure = Constraint เพิ่มเติม\
-   Optimization = กระจาย error ทั้ง graph

------------------------------------------------------------------------

# 🚀 Installation

## Install Dependencies

``` bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-slam-toolbox                  ros-$ROS_DISTRO-nav2-map-server                  ros-$ROS_DISTRO-robot-localization
```

## Build Workspace

``` bash
cd Mobile_Robot
colcon build
source install/setup.bash
```

## Run System

``` bash
ros2 launch lab1 turtlebot.launch.py
rviz2
```

------------------------------------------------------------------------

# 📊 Results

  Method           Drift     Accuracy    Stability
  ---------------- --------- ----------- -------------
  Wheel Odometry   High      Low         Unstable
  EKF Fusion       Medium    Good        Stable
  EKF + ICP        Low       High        Very Stable
  Full SLAM        Minimal   Very High   Optimized

------------------------------------------------------------------------

# 👨‍💻 Author

**Kunanon Sawetkotchakul**\
Robotics & Automation Engineering\
Institute of Field Robotics (FIBO)
