#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Subscribers
        self.sub_curr_xyz = self.create_subscription(Vector3, '/debug/current_xyz', self.curr_xyz_cb, 10)
        self.sub_tgt_xyz = self.create_subscription(Vector3, '/debug/target_xyz', self.tgt_xyz_cb, 10)
        self.mode_sub = self.create_subscription(String, '/set_flight_mode', self.mode_callback, 10)
        
        # --- Experiment Setup ---
        self.is_evaluating = False
        self.eval_duration = 40.0  # ตั้งเวลาให้เท่ากับตัว Performance Metrics (วินาที)
        self.eval_start_time = 0.0
        self.current_flight_mode = "NONE"
        
        # Data Storage
        self.curr_data = {'t': [], 'x': [], 'y': [], 'z': []}
        self.tgt_data = {'t': [], 'x': [], 'y': [], 'z': []}

    def reset_data(self):
        """ล้างข้อมูลกราฟเก่า เพื่อเตรียมพล็อตการทดลองใหม่"""
        self.curr_data = {'t': [], 'x': [], 'y': [], 'z': []}
        self.tgt_data = {'t': [], 'x': [], 'y': [], 'z': []}

    def mode_callback(self, msg):
        new_mode = msg.data.upper()
        
        if new_mode == "IDLE":
            self.is_evaluating = False
            self.get_logger().info("Plotter PAUSED (Mode: IDLE).")
            return
            
        if new_mode != self.current_flight_mode:
            self.current_flight_mode = new_mode
            self.reset_data()
            self.eval_start_time = time.time()
            self.is_evaluating = True
            self.get_logger().info(f"Plotter STARTED recording: {new_mode} for {self.eval_duration}s")

    def curr_xyz_cb(self, msg):
        if not self.is_evaluating:
            return
            
        current_t = time.time() - self.eval_start_time
        
        # ถ้าเกินเวลาที่กำหนด ให้หยุดบันทึกกราฟ
        if current_t > self.eval_duration:
            if self.is_evaluating:
                self.is_evaluating = False
                self.get_logger().info(f"Plotter FINISHED recording for {self.eval_duration}s.")
            return
            
        self.curr_data['t'].append(current_t)
        self.curr_data['x'].append(msg.x)
        self.curr_data['y'].append(msg.y)
        self.curr_data['z'].append(msg.z)

    def tgt_xyz_cb(self, msg):
        if not self.is_evaluating:
            return
            
        current_t = time.time() - self.eval_start_time
        
        if current_t > self.eval_duration:
            return
            
        self.tgt_data['t'].append(current_t)
        self.tgt_data['x'].append(msg.x)
        self.tgt_data['y'].append(msg.y)
        self.tgt_data['z'].append(msg.z)

# ฟังก์ชันสำหรับรัน ROS 2 Spin ใน Background Thread
def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()
    
    # --- ตั้งค่า Matplotlib ---
    fig = plt.figure(figsize=(14, 10))
    ax_x = fig.add_subplot(2, 2, 1)
    ax_y = fig.add_subplot(2, 2, 2)
    ax_z = fig.add_subplot(2, 2, 3)
    ax_3d = fig.add_subplot(2, 2, 4, projection='3d')
    
    def update_plot(frame):
        ax_x.cla(); ax_y.cla(); ax_z.cla(); ax_3d.cla()
        
        status_text = "Recording..." if node.is_evaluating else "Finished/Idle"
        fig.suptitle(f"Flight Mode: {node.current_flight_mode} | Status: {status_text}", fontsize=16)
        
        # ==========================================
        # 1. กราฟ X vs Time
        # ==========================================
        if node.tgt_data['t']: ax_x.plot(node.tgt_data['t'], node.tgt_data['x'], 'g--', label='Target X')
        if node.curr_data['t']: ax_x.plot(node.curr_data['t'], node.curr_data['x'], 'b-', label='Actual X')
        ax_x.set_title('X Position vs Time')
        ax_x.set_xlabel('Time [s]')
        ax_x.set_ylabel('X [m]')
        ax_x.set_xlim([0, node.eval_duration]) # ล็อคแกนเวลาแนวนอน
        ax_x.set_ylim([-10.0, 10.0])           # <--- ล็อคแกนตำแหน่งแนวตั้งของ X
        ax_x.grid(True)
        if node.tgt_data['t'] or node.curr_data['t']:
            ax_x.legend(loc='upper right')
        
        # ==========================================
        # 2. กราฟ Y vs Time
        # ==========================================
        if node.tgt_data['t']: ax_y.plot(node.tgt_data['t'], node.tgt_data['y'], 'g--', label='Target Y')
        if node.curr_data['t']: ax_y.plot(node.curr_data['t'], node.curr_data['y'], 'b-', label='Actual Y')
        ax_y.set_title('Y Position vs Time')
        ax_y.set_xlabel('Time [s]')
        ax_y.set_ylabel('Y [m]')
        ax_y.set_xlim([0, node.eval_duration]) # ล็อคแกนเวลาแนวนอน
        ax_y.set_ylim([-10.0, 10.0])           # <--- ล็อคแกนตำแหน่งแนวตั้งของ Y
        ax_y.grid(True)
        if node.tgt_data['t'] or node.curr_data['t']:
            ax_y.legend(loc='upper right')

        # ==========================================
        # 3. กราฟ Z vs Time
        # ==========================================
        if node.tgt_data['t']: ax_z.plot(node.tgt_data['t'], node.tgt_data['z'], 'g--', label='Target Z')
        if node.curr_data['t']: ax_z.plot(node.curr_data['t'], node.curr_data['z'], 'b-', label='Actual Z')
        ax_z.set_title('Z Position (Altitude) vs Time')
        ax_z.set_xlabel('Time [s]')
        ax_z.set_ylabel('Z [m]')
        ax_z.set_xlim([0, node.eval_duration]) # ล็อคแกนเวลาแนวนอน
        ax_z.set_ylim([0.0, 10.0])             # <--- ล็อคแกนตำแหน่งแนวตั้งของ Z (ความสูงเริ่มจาก 0)
        ax_z.grid(True)
        if node.tgt_data['t'] or node.curr_data['t']:
            ax_z.legend(loc='upper right')

        # ==========================================
        # 4. กราฟ 3D Trajectory
        # ==========================================
        ax_3d.set_xlim([-10.0, 10.0])
        ax_3d.set_ylim([-10.0, 10.0])
        ax_3d.set_zlim([0.0, 10.0])
        ax_3d.set_box_aspect((1, 1, 1))
        ax_3d.set_xlabel('X [m]')
        ax_3d.set_ylabel('Y [m]')
        ax_3d.set_zlabel('Z [m]')
        ax_3d.set_title('UAV 3D Trajectory')
        
        if node.tgt_data['x']:
            ax_3d.plot(node.tgt_data['x'], node.tgt_data['y'], node.tgt_data['z'], 'g--', label='Target', alpha=0.6)
            ax_3d.scatter(node.tgt_data['x'][-1], node.tgt_data['y'][-1], node.tgt_data['z'][-1], color='green', marker='x', s=100)
            
        if node.curr_data['x']:
            ax_3d.plot(node.curr_data['x'], node.curr_data['y'], node.curr_data['z'], 'b-', label='Actual', linewidth=1.5)
            ax_3d.scatter(node.curr_data['x'][-1], node.curr_data['y'][-1], node.curr_data['z'][-1], color='blue', marker='o', s=50)
            
        if node.tgt_data['x'] or node.curr_data['x']:
            ax_3d.legend()
            
        plt.tight_layout()
        
    ani = FuncAnimation(fig, update_plot, interval=200, cache_frame_data=False)
    plt.show() 
    
    node.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=1.0)

if __name__ == '__main__':
    main()