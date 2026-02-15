#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose, PoseStamped 
import math
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy

class ICPNode(Node):
    def __init__(self):
        super().__init__('ICP_node')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            qos_profile
        )
        
        self.create_subscription(PoseStamped, "/turtle_pose_EKF", self.ekf_callback, 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/turtle_pose_ICP", 10)

        # เก็บค่า EKF ปัจจุบัน
        self.ekf_pose = [0.0, 0.0, 0.0]
        
        # ตัวแปรสำหรับระบบ Keyframe เพื่อแก้ปัญหาเส้นหดสั้นในโถงทางเดิน (Corridor Problem)
        self.keyframe_pc = None
        self.keyframe_ekf_pose = [0.0, 0.0, 0.0]
        self.keyframe_icp_pose = [0.0, 0.0, 0.0]
        
        # ตำแหน่ง ICP ล่าสุด
        self.current_icp_pose = [0.0, 0.0, 0.0]

    def ekf_callback(self, msg):
        q = msg.pose.orientation 
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.ekf_pose = [msg.pose.position.x, msg.pose.position.y, yaw]

    def lidar_callback(self, msg):
        current_stamp = msg.header.stamp

        point_x, point_y = [], []
        for i in range(len(msg.ranges)):
            dist = msg.ranges[i]
            if math.isinf(dist) or math.isnan(dist) or dist < msg.range_min or dist > msg.range_max:
                continue
            angle = msg.angle_min + (i * msg.angle_increment)
            point_x.append(dist * math.cos(angle))
            point_y.append(dist * math.sin(angle))

        if len(point_x) < 10: return
        current_pc = np.array([point_x, point_y])

        if self.keyframe_pc is not None:
            # 1. Initial Guess จาก EKF (เทียบกับ Keyframe ล่าสุด ไม่ใช่เฟรมที่แล้ว)
            dx_world = self.ekf_pose[0] - self.keyframe_ekf_pose[0]
            dy_world = self.ekf_pose[1] - self.keyframe_ekf_pose[1]
            dth_guess = self.ekf_pose[2] - self.keyframe_ekf_pose[2]
            
            # [แก้ไข] Normalization ป้องกันมุมกระโดด 360 องศา
            dth_guess = math.atan2(math.sin(dth_guess), math.cos(dth_guess))
            
            cos_p = math.cos(self.keyframe_ekf_pose[2])
            sin_p = math.sin(self.keyframe_ekf_pose[2])
            dx_guess = dx_world * cos_p + dy_world * sin_p
            dy_guess = -dx_world * sin_p + dy_world * cos_p

            T_guess = np.array([
                [math.cos(dth_guess), -math.sin(dth_guess), dx_guess],
                [math.sin(dth_guess),  math.cos(dth_guess), dy_guess],
                [0, 0, 1]
            ])

            # ใช้ T_guess ขยับ current_pc
            p_transformed = T_guess[:2, :2] @ current_pc + T_guess[:2, 2:3]
            final_T = T_guess.copy()

            # 2. วนลูป ICP เทียบกับ Keyframe
            for iteration in range(10):
                indices = self.find_nearest_indices(p_transformed, self.keyframe_pc)
                q_matched = self.keyframe_pc[:, indices]

                diffs = p_transformed - q_matched
                dists = np.linalg.norm(diffs, axis=0)
                
                valid_mask = dists < 0.2
                p_valid = p_transformed[:, valid_mask]
                q_valid = q_matched[:, valid_mask]

                if p_valid.shape[1] < 10:
                    break

                # หา Centroids และทำ SVD
                mu_p = np.mean(p_valid, axis=1).reshape(2, 1)
                mu_q = np.mean(q_valid, axis=1).reshape(2, 1)
                p_prime = p_valid - mu_p
                q_prime = q_valid - mu_q
                
                H = p_prime @ q_prime.T
                U, S, Vt = np.linalg.svd(H)
                R_i = Vt.T @ U.T
                if np.linalg.det(R_i) < 0:
                    Vt[1, :] *= -1
                    R_i = Vt.T @ U.T
                t_i = mu_q - R_i @ mu_p

                # อัปเดตพิกัดจุดและสะสม Matrix
                p_transformed = R_i @ p_transformed + t_i
                
                T_i = np.eye(3)
                T_i[:2, :2] = R_i
                T_i[:2, 2] = t_i.flatten()
                final_T = T_i @ final_T

            # 3. อัปเดต Pose ด้วยระยะขจัดจาก Keyframe
            dx_k = final_T[0, 2]
            dy_k = final_T[1, 2]
            dth_k = math.atan2(final_T[1, 0], final_T[0, 0])

            cos_th_k = math.cos(self.keyframe_icp_pose[2])
            sin_th_k = math.sin(self.keyframe_icp_pose[2])
            
            curr_x = self.keyframe_icp_pose[0] + dx_k * cos_th_k - dy_k * sin_th_k
            curr_y = self.keyframe_icp_pose[1] + dx_k * sin_th_k + dy_k * cos_th_k
            curr_th = self.keyframe_icp_pose[2] + dth_k
            
            # [แก้ไข] Normalization มุมผลลัพธ์สุดท้าย
            curr_th = math.atan2(math.sin(curr_th), math.cos(curr_th))
            
            self.current_icp_pose = [curr_x, curr_y, curr_th]
            self.publish_pose(current_stamp, curr_x, curr_y, curr_th)

            # 4. ตรวจสอบว่าหุ่นยนต์เคลื่อนที่ไปพอสมควรหรือยังเพื่อตั้ง Keyframe ใหม่
            dist_moved = math.hypot(dx_k, dy_k)
            # อัปเดตเมื่อเดินไปมากกว่า 15 ซม. หรือหมุนมากกว่า 0.1 Radian (~5.7 องศา)
            if dist_moved > 0.15 or abs(dth_k) > 0.1:
                self.keyframe_pc = current_pc
                self.keyframe_ekf_pose = self.ekf_pose.copy()
                self.keyframe_icp_pose = self.current_icp_pose.copy()

        else:
            # จับภาพเฟรมแรกสุดตั้งเป็น Keyframe ตั้งต้น
            self.keyframe_pc = current_pc
            self.keyframe_ekf_pose = self.ekf_pose.copy()
            self.keyframe_icp_pose = [0.0, 0.0, 0.0]
            self.current_icp_pose = [0.0, 0.0, 0.0]

    def find_nearest_indices(self, p, q):
        diff = p.T[:, np.newaxis, :] - q.T[np.newaxis, :, :]
        dist_sq = np.sum(diff**2, axis=-1)
        return np.argmin(dist_sq, axis=1)

    def publish_pose(self, stamp, x, y, theta):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)
        self.pose_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ICPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()