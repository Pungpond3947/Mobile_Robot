#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

class TrajectoryPlotter(Node):
    def __init__(self):
        super().__init__('trajectory_plotter')
        
        # Subscribe แค่ตำแหน่ง XYZ ของโดรนและเป้าหมาย
        self.sub_curr_xyz = self.create_subscription(Vector3, '/debug/current_xyz', self.curr_xyz_cb, 10)
        self.sub_tgt_xyz = self.create_subscription(Vector3, '/debug/target_xyz', self.tgt_xyz_cb, 10)
        
        # เก็บประวัติข้อมูลเพื่อนำไปวาดเป็นเส้น
        self.curr_xyz_history = []
        self.tgt_xyz_history = []

    def curr_xyz_cb(self, msg):
        self.curr_xyz_history.append((msg.x, msg.y, msg.z))

    def tgt_xyz_cb(self, msg):
        self.tgt_xyz_history.append((msg.x, msg.y, msg.z))

# ฟังก์ชันสำหรับรัน ROS 2 Spin ใน Background Thread
def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlotter()
    
    # แยก Thread ให้ ROS 2 ทำงานรับข้อมูลเบื้องหลัง
    thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    thread.start()
    
    # --- ตั้งค่า Matplotlib ---
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')
    
    def update_plot(frame):
        ax.cla() # ล้างกราฟเก่าก่อนวาดเฟรมใหม่
        
        ax.set_xlabel('X (North)')
        ax.set_ylabel('Y (West)')
        ax.set_zlabel('Z (Up)')
        ax.set_title('UAV 3D Trajectory (Actual vs Target)')
        
        # 1. พล็อตเส้นทางเป้าหมาย (Target) - เส้นประสีเขียว
        if node.tgt_xyz_history:
            tx, ty, tz = zip(*node.tgt_xyz_history)
            ax.plot(tx, ty, tz, 'g--', label='Target Path', alpha=0.6)
            # มาร์คจุดเป้าหมายล่าสุดด้วยกากบาทสีเขียว
            ax.scatter(tx[-1], ty[-1], tz[-1], color='green', marker='x', s=100, label='Current Target')
            
        # 2. พล็อตเส้นทางจริงของโดรน (Actual) - เส้นทึบสีน้ำเงิน
        if node.curr_xyz_history:
            cx, cy, cz = zip(*node.curr_xyz_history)
            ax.plot(cx, cy, cz, 'b-', label='Actual Path', linewidth=1.5)
            # มาร์คจุดโดรนล่าสุดด้วยวงกลมสีน้ำเงิน
            ax.scatter(cx[-1], cy[-1], cz[-1], color='blue', marker='o', s=50, label='Current Position')
            
        # แสดง Legend
        if node.tgt_xyz_history or node.curr_xyz_history:
            ax.legend()
        
    # อัปเดตกราฟทุกๆ 200 มิลลิวินาที
    ani = FuncAnimation(fig, update_plot, interval=200, cache_frame_data=False)
    
    plt.show() # รันกราฟ
    
    # ปิดการทำงานเมื่อหน้าต่างกราฟถูกปิด
    node.destroy_node()
    rclpy.shutdown()
    thread.join(timeout=1.0)

if __name__ == '__main__':
    main()