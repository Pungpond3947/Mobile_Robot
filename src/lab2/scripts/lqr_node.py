#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
import math
import numpy as np
from scipy.linalg import solve_continuous_are

def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z

class LQRiController(Node):
    def __init__(self):
        super().__init__('lqri_controller')
        
        # --- Subscribers & Publishers ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.motor_pub = self.create_publisher(Actuators, '/motor_commands', 10)

        # --- Debug Publishers ---
        self.pub_curr_z = self.create_publisher(Float64, '/debug/current_z', 10)
        self.pub_tgt_z = self.create_publisher(Float64, '/debug/target_z', 10)
        self.pub_curr_rpy = self.create_publisher(Vector3, '/debug/current_rpy', 10)
        self.pub_tgt_rpy = self.create_publisher(Vector3, '/debug/target_rpy', 10)

        # --- Targets ---
        self.target_z = 2.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # ==========================================
        # 1. ข้อมูลทางฟิสิกส์จาก URDF
        # ==========================================
        self.mass = 1.5
        self.gravity = 9.81
        self.I_xx = 0.0347563
        self.I_yy = 0.07
        self.I_zz = 0.0977
        self.k_F = 8.54858e-06
        self.k_M = 0.06
        self.L_x = 0.13
        self.L_y = 0.22
        self.omega_max = 1500.0
        
        # ==========================================
        # 2. Motor Mixing Matrix
        # ==========================================
        self.M = np.array([
            [ 1.0,       1.0,      1.0,       1.0],       # Total Thrust
            [-self.L_y,  self.L_y, self.L_y, -self.L_y],  # Roll Torque
            [-self.L_x,  self.L_x,-self.L_x,  self.L_x],  # Pitch Torque
            [-self.k_M, -self.k_M, self.k_M,  self.k_M]   # Yaw Torque
        ])
        self.M_inv = np.linalg.inv(self.M) 
        
        # ==========================================
        # 3. คำนวณ LQRi Gain Matrix
        # ==========================================
        self.calculate_lqri_gains()
        
        # ==========================================
        # 4. Z-Controller (PID)
        # ==========================================
        self.kp_z = 12.0
        self.ki_z = 2.0
        self.kd_z = 8.0
        self.integral_z = 0.0
        self.prev_error_z = 0.0
        self.max_i_z = 10.0

        # --- States ปัจจุบัน ---
        self.curr_z = 0.0
        self.curr_roll = 0.0; self.curr_pitch = 0.0; self.curr_yaw = 0.0
        self.curr_p = 0.0; self.curr_q = 0.0; self.curr_r = 0.0
        self.odom_ready = False
        
        # --- LQR Integral States ---
        self.err_int_roll = 0.0
        self.err_int_pitch = 0.0
        self.err_int_yaw = 0.0
        self.max_i_att = 2.0 
        
        self.last_time = self.get_clock().now()
        
        # --- Timer 100 Hz ---
        self.control_timer = self.create_timer(0.01, self.control_loop) 

    def calculate_lqri_gains(self):
        A = np.zeros((9, 9))
        A[0, 3] = 1.0; A[1, 4] = 1.0; A[2, 5] = 1.0
        A[3, 6] = 1.0; A[4, 7] = 1.0; A[5, 8] = 1.0
        
        B = np.zeros((9, 3))
        B[6, 0] = 1.0 / self.I_xx
        B[7, 1] = 1.0 / self.I_yy
        B[8, 2] = 1.0 / self.I_zz
        
        Q = np.diag([0.001, 0.002, 0.001, 0.5, 0.5, 0.5, 0.0005, 0.0005, 0.0005])
        R = np.diag([0.1, 0.1, 0.1])
        
        P = solve_continuous_are(A, B, Q, R)
        self.K_lqri = np.linalg.inv(R).dot(B.T).dot(P)
        self.get_logger().info('LQRi Gains Calculated with Debug Topics enabled.')

    def odom_callback(self, msg):
        self.curr_z = msg.pose.pose.position.z
        q = msg.pose.pose.orientation
        self.curr_roll, self.curr_pitch, self.curr_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        self.curr_p = msg.twist.twist.angular.x
        self.curr_q = msg.twist.twist.angular.y
        self.curr_r = msg.twist.twist.angular.z
        
        self.odom_ready = True

        # --- Publish Debug Data (ส่งเป็น Radians โดยตรง) ---
        self.pub_curr_z.publish(Float64(data=self.curr_z))
        self.pub_tgt_z.publish(Float64(data=self.target_z))
        
        curr_rpy_msg = Vector3(
            x=self.curr_roll, 
            y=self.curr_pitch, 
            z=self.curr_yaw
        )
        self.pub_curr_rpy.publish(curr_rpy_msg)
        
        tgt_rpy_msg = Vector3(
            x=self.target_roll, 
            y=self.target_pitch, 
            z=self.target_yaw
        )
        self.pub_tgt_rpy.publish(tgt_rpy_msg)

    def control_loop(self):
        if not self.odom_ready:
            return

        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        # 1. Z-Controller
        error_z = self.target_z - self.curr_z
        self.integral_z = np.clip(self.integral_z + error_z * dt, -self.max_i_z, self.max_i_z)
        deriv_z = (error_z - self.prev_error_z) / dt
        self.prev_error_z = error_z
        
        base_thrust = self.mass * self.gravity
        z_thrust_cmd = (self.kp_z * error_z) + (self.ki_z * self.integral_z) + (self.kd_z * deriv_z)
        total_thrust = base_thrust + z_thrust_cmd

        # 2. LQRi Attitude Controller
        error_roll = self.curr_roll - self.target_roll
        error_pitch = self.curr_pitch - self.target_pitch
        error_yaw = self.curr_yaw - self.target_yaw
        
        self.err_int_roll = np.clip(self.err_int_roll + error_roll * dt, -self.max_i_att, self.max_i_att)
        self.err_int_pitch = np.clip(self.err_int_pitch + error_pitch * dt, -self.max_i_att, self.max_i_att)
        self.err_int_yaw = np.clip(self.err_int_yaw + error_yaw * dt, -self.max_i_att, self.max_i_att)
        
        x_aug = np.array([
            self.err_int_roll, self.err_int_pitch, self.err_int_yaw,
            error_roll, error_pitch, error_yaw,
            self.curr_p, self.curr_q, self.curr_r
        ])
        
        u = -np.dot(self.K_lqri, x_aug)
        tau_x, tau_y, tau_z = u[0], u[1], u[2]

        # 3. Motor Mixing
        forces = np.array([total_thrust, tau_x, tau_y, tau_z])
        T_motors = np.dot(self.M_inv, forces)
        
        w_cmds = [0.0] * 4
        for i in range(4):
            thrust_i = max(0.0, T_motors[i]) 
            w = math.sqrt(thrust_i / self.k_F)
            w_cmds[i] = min(self.omega_max, float(w))

        msg_out = Actuators()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.velocity = w_cmds
        self.motor_pub.publish(msg_out)

        self.last_time = curr_time

def main(args=None):
    rclpy.init(args=args)
    node = LQRiController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()