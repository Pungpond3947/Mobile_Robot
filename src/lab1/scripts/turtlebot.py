#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import numpy as np
from numpy.linalg import inv

class TurtlebotNode(Node):
    def __init__(self):
        super().__init__('turtlebot_node')

        self.create_subscription(JointState, "/joint_states", self.joint_states_callback, 10)
        self.create_subscription(Imu, "/imu", self.imu_callback, 10)

        self.pose_pub = self.create_publisher(PoseStamped, "/turtle_pose_EKF", 10)
        self.wheel_pub = self.create_publisher(PoseStamped, "/turtle_pose_wheel", 10)

        # เก็บไว้เผื่ออ้างอิง แต่ในโค้ดนี้จะไม่ได้ใช้คูณความเร็วแล้ว
        self.R = 0.033 # รัศมีล้อ Turtlebot ปกติจะอยู่ราวๆ 3.3 ซม.
        self.L = 0.160 # Track width ระยะห่างล้อซ้ายขวา

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.theta = 0.0

        self.wheel_x = 0.0
        self.wheel_y = 0.0
        self.wheel_theta = 0.0

        self.v = 0.0
        self.omega = 0.0
        self.last_timestamp = None
        
        self.initial_yaw = None 

        # Matrix สำหรับ EKF
        self.state_predict = np.zeros((3, 1))
        self.state_update = np.zeros((3, 1))
        self.Matrix_P = np.diag([0.1, 0.1, 0.1])
        self.Matrix_Q = np.diag([0.8, 0.8, 0.8])
        self.Matrix_R = np.zeros((1, 1))
        self.Jaco_Matrix = np.zeros((3, 3))
        self.Kalman_Gain = np.zeros((3, 1))
        self.Matrix_Measurment = np.zeros((1, 3))

        self.Jaco_Matrix[0][0] = 1.0
        self.Jaco_Matrix[1][1] = 1.0
        self.Jaco_Matrix[2][2] = 1.0

        self.Matrix_Measurment[0][2] = 1.0
        self.Matrix_R[0][0] = 0.3

    def imu_callback(self, msg):
        current_stamp = msg.header.stamp

        q = msg.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw_imu = math.atan2(siny_cosp, cosy_cosp)
        
        if self.initial_yaw is None:
            self.initial_yaw = yaw_imu
        
        yaw_imu = yaw_imu - self.initial_yaw
        yaw_imu = math.atan2(math.sin(yaw_imu), math.cos(yaw_imu))

        # Kalman Correction
        x = (self.Matrix_Measurment @ self.Matrix_P @ self.Matrix_Measurment.T + self.Matrix_R)
        self.Kalman_Gain = self.Matrix_P @ self.Matrix_Measurment.T @ inv(x)

        error = yaw_imu - self.state_predict[2][0]
        error = math.atan2(math.sin(error), math.cos(error)) 
        
        self.state_update = self.state_predict + (self.Kalman_Gain * error)
        I = np.eye(3)
        self.Matrix_P = (I - self.Kalman_Gain @ self.Matrix_Measurment) @ self.Matrix_P

        self.pos_x = float(self.state_update[0][0])
        self.pos_y = float(self.state_update[1][0])
        self.theta = float(self.state_update[2][0])
        self.state_predict = self.state_update.copy()

        self.publish_ekf_pose(current_stamp)

    def joint_states_callback(self, msg):
        current_stamp = msg.header.stamp
        current_time = current_stamp.sec + current_stamp.nanosec * 1e-9
        
        if self.last_timestamp is None:
            self.last_timestamp = current_time
            return
        dt = current_time - self.last_timestamp
        if dt <= 0: return
        self.last_timestamp = current_time

        try:
            l_idx = msg.name.index('wheel_left_joint')
            r_idx = msg.name.index('wheel_right_joint')
        except ValueError:
            return

        # [แก้ไขแล้ว] ถอด * self.R ออก เนื่องจากค่ามาเป็น m/s แล้ว
        v_l = msg.velocity[l_idx] 
        v_r = msg.velocity[r_idx] 
        
        self.v = (v_r + v_l) / 2.0
        self.omega = (v_r - v_l) / self.L

        # Prediction
        theta_prev_ekf = self.theta
        self.theta += self.omega * dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        self.pos_x += self.v * math.cos(theta_prev_ekf) * dt
        self.pos_y += self.v * math.sin(theta_prev_ekf) * dt

        self.state_predict[0][0] = self.pos_x
        self.state_predict[1][0] = self.pos_y
        self.state_predict[2][0] = self.theta

        theta_prev_wheel = self.wheel_theta
        self.wheel_theta += self.omega * dt
        self.wheel_theta = math.atan2(math.sin(self.wheel_theta), math.cos(self.wheel_theta))
        self.wheel_x += self.v * math.cos(theta_prev_wheel) * dt
        self.wheel_y += self.v * math.sin(theta_prev_wheel) * dt
        
        self.publish_wheel_pose(current_stamp)

        # EKF Covariance Update
        d_k = self.v * dt
        self.Jaco_Matrix[0][2] = -d_k * math.sin(theta_prev_ekf)
        self.Jaco_Matrix[1][2] =  d_k * math.cos(theta_prev_ekf)
        self.Matrix_P = self.Jaco_Matrix @ self.Matrix_P @ self.Jaco_Matrix.T + self.Matrix_Q

    def euler_to_quaternion(self, yaw):
        return Quaternion(x=0.0, y=0.0, z=math.sin(yaw / 2.0), w=math.cos(yaw / 2.0))

    def publish_ekf_pose(self, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        
        msg.pose.position.x = self.pos_x
        msg.pose.position.y = self.pos_y
        msg.pose.orientation = self.euler_to_quaternion(self.theta)
        self.pose_pub.publish(msg)

    def publish_wheel_pose(self, stamp):
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = "odom"
        
        msg.pose.position.x = self.wheel_x
        msg.pose.position.y = self.wheel_y
        msg.pose.orientation = self.euler_to_quaternion(self.wheel_theta)
        self.wheel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()