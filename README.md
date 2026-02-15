# 🤖 LAB 1: Mobile Robot SLAM & Sensor Fusion

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/index.html)
[![Python](https://img.shields.io/badge/Language-Python-yellow)](https://www.python.org/)

โปรเจกต์นี้เป็นการพัฒนาระบบ Localization และ Mapping สำหรับหุ่นยนต์เคลื่อนที่ 2 ล้อ (Differential Drive) เพื่อศึกษาและเปรียบเทียบประสิทธิภาพของการฟิวชันเซนเซอร์ (Sensor Fusion), การจับคู่สแกน (Scan Matching), และการสร้างแผนที่ (SLAM) ในการลดความคลาดเคลื่อนสะสม (Drift)

---

## 🎯 วัตถุประสงค์ (Learning Outcomes)
1. **Part 1 (EKF Odometry Fusion):** ใช้งาน Extended Kalman Filter เพื่อฟิวชัน Wheel Odometry เข้ากับข้อมูล IMU
2. **Part 2 (ICP Odometry Refinement):** ใช้งาน Iterative Closest Point เพื่อปรับปรุง Odometry จากข้อมูล LiDAR
3. **Part 3 (Full SLAM):** สร้างแผนที่ 2D และแก้ไข Error สะสมด้วยระบบ Loop Closure ของ `slam_toolbox`

---

## 🛠 โครงสร้างระบบ (System Architecture)

ระบบประกอบด้วย 4 ส่วนหลักที่ทำงานประสานกันผ่าน ROS2:

* **`turtlebot.py`:** โหนดคำนวณ Differential Drive Kinematics และทำ EKF (Prediction & Update step)
* **`icp_ekf.py`:** โหนดทำ Point-to-Point Scan Matching โดยอ้างอิง Initial Guess จาก EKF 
* **`turtlebot_pose.py`:** โหนดจัดการ TF Tree และสร้าง `nav_msgs/Path` เพื่อแสดงผล
* **`slam_path.py`:** โหนดติดตาม Pose ที่ปรับแก้แล้วจาก SLAM บน Map Frame

---

## 🚀 วิธีการติดตั้งและใช้งาน (Usage)

### 1. ติดตั้ง Dependencies
โปรเจกต์นี้จำเป็นต้องใช้แพ็กเกจสำหรับการทำแผนที่:
```bash
sudo apt update
sudo apt install ros-$ROS_DISTRO-slam-toolbox ros-$ROS_DISTRO-nav2-map-server

จากอันนี้เติมวิธีการรันให้ด้วย
