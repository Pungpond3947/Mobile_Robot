#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from actuator_msgs.msg import Actuators
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64, String
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

class LQRiTrajectoryController(Node):
    def __init__(self):
        super().__init__('lqri_trajectory_controller')
        
        # --- Subscribers & Publishers ---
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.imu_callback, 10) 
        self.motor_pub = self.create_publisher(Actuators, '/motor_commands', 10)

        self.target_sub = self.create_subscription(Vector3, '/set_target_xyz', self.target_callback, 10)
        self.mode_sub = self.create_subscription(String, '/set_flight_mode', self.mode_callback, 10)

        self.pub_curr_xyz = self.create_publisher(Vector3, '/debug/current_xyz', 10)
        self.pub_tgt_xyz = self.create_publisher(Vector3, '/debug/target_xyz', 10)
        self.pub_curr_rpy = self.create_publisher(Vector3, '/debug/current_rpy', 10)
        self.pub_tgt_rpy = self.create_publisher(Vector3, '/debug/target_rpy', 10)

        # ==========================================
        # Flight States
        # ==========================================
        self.motors_active = False    
        self.flight_mode = "NONE"     
        self.trajectory_type = "NONE" 
        
        self.mode_start_time = None
        
        self.traj_start_x = 0.0
        self.traj_start_y = 0.0
        self.traj_start_z = 2.0
        
        # --- Targets ---
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 2.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0
        
        # ==========================================
        # 1. ข้อมูลทางฟิสิกส์
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
            [ 1.0,       1.0,      1.0,       1.0],       
            [-self.L_y,  self.L_y, self.L_y, -self.L_y],  
            [-self.L_x,  self.L_x,-self.L_x,  self.L_x],  
            [-self.k_M, -self.k_M, self.k_M,  self.k_M]   
        ])
        self.M_inv = np.linalg.inv(self.M) 
        
        # ==========================================
        # 3. LQRi & Z-PID
        # ==========================================
        self.calculate_lqri_gains()
        self.kp_z = 12.0; self.ki_z = 2.0; self.kd_z = 8.0
        self.integral_z = 0.0; self.prev_error_z = 0.0; self.max_i_z = 10.0

        # ==========================================
        # 4. X-Y Position Controller (PID)
        # ==========================================
        self.kp_pos = 0.5
        self.ki_pos = 0.07
        self.kd_pos = 0.25
        
        self.integral_x = 0.0; self.prev_error_x = 0.0
        self.integral_y = 0.0; self.prev_error_y = 0.0
        self.max_i_pos = 2.0
        self.max_angle_cmd = math.radians(25) 

        # --- States ปัจจุบัน ---
        self.curr_x = 0.0; self.curr_y = 0.0; self.curr_z = 0.0
        self.curr_roll = 0.0; self.curr_pitch = 0.0; self.curr_yaw = 0.0
        self.curr_p = 0.0; self.curr_q = 0.0; self.curr_r = 0.0
        
        self.odom_ready = False
        self.imu_ready = False
        
        self.err_int_roll = 0.0; self.err_int_pitch = 0.0; self.err_int_yaw = 0.0
        self.max_i_att = 2.0 
        self.last_time = self.get_clock().now()
        self.control_timer = self.create_timer(0.01, self.control_loop) 

    def calculate_lqri_gains(self):
        A = np.zeros((9, 9)); B = np.zeros((9, 3))
        A[0, 3] = 1.0; A[1, 4] = 1.0; A[2, 5] = 1.0
        A[3, 6] = 1.0; A[4, 7] = 1.0; A[5, 8] = 1.0
        B[6, 0] = 1.0 / self.I_xx; B[7, 1] = 1.0 / self.I_yy; B[8, 2] = 1.0 / self.I_zz
        Q = np.diag([0.001, 0.002, 0.001, 0.5, 0.5, 0.5, 0.0005, 0.0005, 0.0005])
        R = np.diag([0.01, 0.01, 0.01])
        P = solve_continuous_are(A, B, Q, R)
        self.K_lqri = np.linalg.inv(R).dot(B.T).dot(P)
        self.get_logger().info('System Initialized. Waiting for Mode Command...')

    def odom_callback(self, msg):
        self.curr_x = msg.pose.pose.position.x
        self.curr_y = msg.pose.pose.position.y
        self.curr_z = msg.pose.pose.position.z
        
        self.odom_ready = True
        
        self.pub_curr_xyz.publish(Vector3(x=self.curr_x, y=self.curr_y, z=self.curr_z))
        self.pub_tgt_xyz.publish(Vector3(x=self.target_x, y=self.target_y, z=self.target_z))

    def imu_callback(self, msg):
        q = msg.orientation
        self.curr_roll, self.curr_pitch, self.curr_yaw = euler_from_quaternion(q.x, q.y, q.z, q.w)
        
        self.curr_p = msg.angular_velocity.x
        self.curr_q = msg.angular_velocity.y
        self.curr_r = msg.angular_velocity.z
        
        self.imu_ready = True
        
        self.pub_curr_rpy.publish(Vector3(x=self.curr_roll, y=self.curr_pitch, z=self.curr_yaw))
        self.pub_tgt_rpy.publish(Vector3(x=self.target_roll, y=self.target_pitch, z=self.target_yaw))

    def target_callback(self, msg):
        if not self.motors_active:
            self.get_logger().warn("Cannot set target. Drone is IDLE. Set mode to 2D or 3D first.")
            return

        self.trajectory_type = "GOTO"
        
        if self.flight_mode == "2D":
            self.target_x = msg.x
            self.target_z = msg.z
            self.get_logger().info(f"2D GOTO -> X: {msg.x:.2f}, Z: {msg.z:.2f} (Y axis ignored)")
            
        elif self.flight_mode == "3D":
            self.target_x = msg.x
            self.target_y = msg.y
            self.target_z = msg.z
            self.get_logger().info(f"3D GOTO -> X: {msg.x:.2f}, Y: {msg.y:.2f}, Z: {msg.z:.2f}")

    def mode_callback(self, msg):
        new_mode = msg.data.upper()
        
        if new_mode == "IDLE":
            self.motors_active = False
            self.flight_mode = "NONE"
            self.trajectory_type = "NONE"
            self.get_logger().info("Mode -> IDLE (Motors OFF)")
            return

        if not self.motors_active:
            self.motors_active = True
            self.traj_start_x = self.curr_x
            self.traj_start_y = self.curr_y
            self.traj_start_z = 2.0  
            
            self.target_x = self.curr_x
            self.target_y = self.curr_y
            self.target_z = 2.0
            
            self.integral_x = 0.0; self.integral_y = 0.0; self.integral_z = 0.0
            self.err_int_roll = 0.0; self.err_int_pitch = 0.0; self.err_int_yaw = 0.0
            self.get_logger().info("Taking off to 2.0m Hover!")

        if new_mode == "2D":
            self.flight_mode = "2D"
            self.trajectory_type = "HOVER"
            self.get_logger().info("Mode -> 2D (Waiting for target X, Z...)")
            
        elif new_mode == "3D":
            self.flight_mode = "3D"
            self.trajectory_type = "HOVER"
            self.get_logger().info("Mode -> 3D (Waiting for target X, Y, Z...)")
            
        elif new_mode == "2D_SINE":
            self.flight_mode = "2D"
            self.trajectory_type = "SINE"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 2D SINE WAVE")
            
        elif new_mode == "2D_STRAIGHT_F":
            self.flight_mode = "2D"
            self.trajectory_type = "STRAIGHT_F"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 2D STRAIGHT FORWARD (+3 m/s)")
            
        elif new_mode == "2D_STRAIGHT_B":
            self.flight_mode = "2D"
            self.trajectory_type = "STRAIGHT_B"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 2D STRAIGHT BACKWARD (-3 m/s)")

        elif new_mode == "2D_RAMP_WAVE":
            self.flight_mode = "2D"
            self.trajectory_type = "RAMP_WAVE"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 2D RAMP WAVE (Triangle Zigzag)")
            
        elif new_mode == "3D_HELIX":
            self.flight_mode = "3D"
            self.trajectory_type = "HELIX"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 3D HELIX")
            
        elif new_mode == "3D_STRAIGHT":
            self.flight_mode = "3D"
            self.trajectory_type = "STRAIGHT_3D"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 3D STRAIGHT LINE")
            
        elif new_mode == "3D_FIGURE8":
            self.flight_mode = "3D"
            self.trajectory_type = "FIGURE8_3D"
            self.mode_start_time = self.get_clock().now()
            self.get_logger().info("Mode -> 3D FIGURE-8")
            
        else:
            self.get_logger().warn("Unknown mode! Use: IDLE, 2D, 3D, 2D_SINE, 2D_STRAIGHT_F, 2D_STRAIGHT_B, 3D_HELIX, 2D_RAMP_WAVE, 3D_STRAIGHT, 3D_FIGURE8")

    def generate_trajectory(self, current_time):
        if self.trajectory_type in ["NONE", "HOVER", "GOTO"]:
            return 
            
        if self.mode_start_time is None:
            self.mode_start_time = current_time
            
        t_traj = (current_time - self.mode_start_time).nanoseconds / 1e9
        
        if self.trajectory_type == "SINE":
            self.target_x = self.traj_start_x + 0.3 * t_traj
            self.target_z = self.traj_start_z + 1.0 * math.sin(0.5 * t_traj)
            
        elif self.trajectory_type == "STRAIGHT_F":
            self.target_x = self.traj_start_x + 0.25 * t_traj 
            self.target_z = self.traj_start_z
            
        elif self.trajectory_type == "STRAIGHT_B":
            self.target_x = self.traj_start_x - 0.25 * t_traj
            self.target_z = self.traj_start_z

        elif self.trajectory_type == "RAMP_WAVE":
            period = 4.0
            phase = (t_traj % period) / period
            
            if phase < 0.5:
                z_wave = 2.0 * phase 
            else:
                z_wave = 2.0 * (1.0 - phase)
                
            self.target_x = self.traj_start_x + 0.25 * t_traj
            self.target_z = self.traj_start_z + (1.5 * z_wave)

        elif self.trajectory_type == "STRAIGHT_3D":
            vx, vy, vz = 0.25, 0.25, 0.25
            self.target_x = self.traj_start_x + vx * t_traj
            self.target_y = self.traj_start_y + vy * t_traj
            self.target_z = self.traj_start_z + vz * t_traj
            
        elif self.trajectory_type == "FIGURE8_3D":
            a = 3.0
            omega = 0.5
            denom = 1.0 + math.sin(omega * t_traj)**2
            
            x_lem = (a * math.cos(omega * t_traj)) / denom
            y_lem = (a * math.sin(omega * t_traj) * math.cos(omega * t_traj)) / denom
            
            self.target_x = self.traj_start_x + x_lem - a
            self.target_y = self.traj_start_y + y_lem
            self.target_z = self.traj_start_z + 1.0 * math.sin(omega * t_traj * 2.0)
            
        elif self.trajectory_type == "HELIX":
            radius = 2.0
            omega = 0.5
            self.target_x = self.traj_start_x + radius * math.sin(omega * t_traj)
            self.target_y = self.traj_start_y + radius * (1.0 - math.cos(omega * t_traj)) 
            self.target_z = self.traj_start_z + 0.1 * t_traj

    def control_loop(self):
        if not (self.odom_ready and self.imu_ready):
            return

        curr_time = self.get_clock().now()
        dt = (curr_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return
        self.last_time = curr_time

        if not self.motors_active:
            msg_out = Actuators()
            msg_out.header.stamp = curr_time.to_msg()
            msg_out.velocity = [0.0, 0.0, 0.0, 0.0]
            self.motor_pub.publish(msg_out)
            return

        self.generate_trajectory(curr_time)

        # ==========================================
        # X-Y Position Controller (Outer Loop)
        # ==========================================
        error_x = self.target_x - self.curr_x
        self.integral_x = np.clip(self.integral_x + error_x * dt, -self.max_i_pos, self.max_i_pos)
        deriv_x = (error_x - self.prev_error_x) / dt
        self.prev_error_x = error_x
        
        cmd_pitch = (self.kp_pos * error_x) + (self.ki_pos * self.integral_x) + (self.kd_pos * deriv_x)
        self.target_pitch = np.clip(cmd_pitch, -self.max_angle_cmd, self.max_angle_cmd)

        if self.flight_mode == "3D":
            error_y = self.target_y - self.curr_y
            self.integral_y = np.clip(self.integral_y + error_y * dt, -self.max_i_pos, self.max_i_pos)
            deriv_y = (error_y - self.prev_error_y) / dt
            self.prev_error_y = error_y
            
            cmd_roll = -(self.kp_pos * error_y) - (self.ki_pos * self.integral_y) - (self.kd_pos * deriv_y)
            self.target_roll = np.clip(cmd_roll, -self.max_angle_cmd, self.max_angle_cmd)
        else:
            self.target_roll = 0.0
            self.integral_y = 0.0
            self.prev_error_y = 0.0

        # ==========================================
        # Z-Controller (Altitude)
        # ==========================================
        error_z = self.target_z - self.curr_z
        self.integral_z = np.clip(self.integral_z + error_z * dt, -self.max_i_z, self.max_i_z)
        deriv_z = (error_z - self.prev_error_z) / dt
        self.prev_error_z = error_z
        
        base_thrust = self.mass * self.gravity
        z_thrust_cmd = (self.kp_z * error_z) + (self.ki_z * self.integral_z) + (self.kd_z * deriv_z)
        total_thrust = base_thrust + z_thrust_cmd

        # ==========================================
        # LQRi Attitude Controller (Inner Loop)
        # ==========================================
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

        # ==========================================
        # Motor Mixing
        # ==========================================
        forces = np.array([total_thrust, tau_x, tau_y, tau_z])
        T_motors = np.dot(self.M_inv, forces)
        
        w_cmds = [0.0] * 4
        for i in range(4):
            thrust_i = max(0.0, T_motors[i]) 
            w = math.sqrt(thrust_i / self.k_F)
            w_cmds[i] = min(self.omega_max, float(w))

        msg_out = Actuators()
        msg_out.header.stamp = curr_time.to_msg()
        msg_out.velocity = w_cmds
        self.motor_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = LQRiTrajectoryController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()