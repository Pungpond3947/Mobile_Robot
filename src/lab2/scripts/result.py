#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
import math

class AxisTracker:
    """คลาสผู้ช่วยสำหรับคำนวณ Metrics แยกตามแกน (ทั้ง Position และ Attitude)"""
    def __init__(self, name, settle_threshold, step_threshold, settle_time_req=0.5):
        self.name = name
        
        # --- Cumulative Metrics (MSE, RMSE) ---
        self.n_samples = 0
        self.sum_sq_err = 0.0
        self.rmse = 0.0
        self.mse = 0.0
        
        # --- Transient Response Metrics ---
        self.tracking_step = False
        self.start_val = 0.0
        self.target_val = 0.0
        self.step_start_time = 0.0
        
        self.max_val_reached = 0.0
        self.overshoot_pct = 0.0
        
        self.step_threshold = step_threshold      # ขนาดของการเปลี่ยน Target ที่จะถือว่าเป็น Step Input
        self.settle_threshold = settle_threshold  # Error ที่ยอมรับว่า "นิ่งแล้ว" 
        self.settle_time_req = settle_time_req    # ต้องอยู่ในระยะ Settle นานเท่าไหร่ (วินาที)
        
        self.time_entered_settle = None
        self.is_settled = False
        self.settling_time = 0.0
        self.ss_error = 0.0

    def check_step_input(self, current_target, current_pos, current_time):
        """ตรวจจับว่ามีการเปลี่ยนเป้าหมายกะทันหันเกิน Threshold หรือไม่"""
        if abs(current_target - self.target_val) > self.step_threshold:
            self.tracking_step = True
            self.start_val = current_pos
            self.target_val = current_target
            self.step_start_time = current_time
            self.max_val_reached = current_pos
            
            self.is_settled = False
            self.time_entered_settle = None
            self.overshoot_pct = 0.0
            self.settling_time = 0.0
            self.ss_error = 0.0
            return True
        return False

    def update(self, current_pos, current_target, current_time):
        # 1. Update MSE / RMSE
        error = current_target - current_pos
        self.sum_sq_err += error**2
        self.n_samples += 1
        self.mse = self.sum_sq_err / self.n_samples
        self.rmse = math.sqrt(self.mse)
        
        # 2. Update Transient Response
        if self.tracking_step:
            step_size = self.target_val - self.start_val
            
            # --- อัปเดต Overshoot ---
            if step_size > 0 and current_pos > self.max_val_reached:
                self.max_val_reached = current_pos
            elif step_size < 0 and current_pos < self.max_val_reached:
                self.max_val_reached = current_pos
                
            if abs(step_size) > 0.0001:
                overshoot_val = abs(self.max_val_reached - self.target_val)
                if (step_size > 0 and self.max_val_reached < self.target_val) or \
                   (step_size < 0 and self.max_val_reached > self.target_val):
                    self.overshoot_pct = 0.0
                else:
                    self.overshoot_pct = (overshoot_val / abs(step_size)) * 100.0

            # --- อัปเดต Settling Time และ Steady-State Error ---
            if not self.is_settled:
                if abs(error) <= self.settle_threshold:
                    if self.time_entered_settle is None:
                        self.time_entered_settle = current_time
                    elif (current_time - self.time_entered_settle) >= self.settle_time_req:
                        self.is_settled = True
                        self.settling_time = current_time - self.step_start_time
                        self.ss_error = error
                else:
                    self.time_entered_settle = None


class PerformanceMetricsNode(Node):
    def __init__(self):
        super().__init__('performance_metrics_node')
        
        # --- Subscribers (XYZ & RPY) ---
        self.sub_curr_xyz = self.create_subscription(Vector3, '/debug/current_xyz', self.curr_xyz_cb, 10)
        self.sub_tgt_xyz = self.create_subscription(Vector3, '/debug/target_xyz', self.tgt_xyz_cb, 10)
        
        self.sub_curr_rpy = self.create_subscription(Vector3, '/debug/current_rpy', self.curr_rpy_cb, 10)
        self.sub_tgt_rpy = self.create_subscription(Vector3, '/debug/target_rpy', self.tgt_rpy_cb, 10)

        # --- Trackers ตำแหน่ง XYZ (เมตร) ---
        # นิ่งเมื่อ Error < 0.05m (5cm), ถือเป็น Step เมื่อเป้าเปลี่ยน > 0.1m
        self.tracker_x = AxisTracker('X-Pos', settle_threshold=0.05, step_threshold=0.1)
        self.tracker_y = AxisTracker('Y-Pos', settle_threshold=0.05, step_threshold=0.1)
        self.tracker_z = AxisTracker('Z-Pos', settle_threshold=0.05, step_threshold=0.1)

        # --- Trackers มุม RPY (เรเดียน) ---
        # นิ่งเมื่อ Error < 0.035 rad (~2 องศา), ถือเป็น Step เมื่อเป้าเปลี่ยน > 0.05 rad (~2.8 องศา)
        self.tracker_roll = AxisTracker('Roll', settle_threshold=0.035, step_threshold=0.05)
        self.tracker_pitch = AxisTracker('Pitch', settle_threshold=0.035, step_threshold=0.05)
        self.tracker_yaw = AxisTracker('Yaw', settle_threshold=0.035, step_threshold=0.05)

        self.curr_xyz = None; self.tgt_xyz = None
        self.curr_rpy = None; self.tgt_rpy = None
        
        # Loop ความถี่ 50Hz (0.02s) 
        self.timer = self.create_timer(0.02, self.metrics_loop)
        
        # Timer สำหรับ Print Report ทุกๆ 2 วินาที
        self.report_timer = self.create_timer(2.0, self.print_report)
        
        self.get_logger().info("Performance Metrics Node Initialized (XYZ & RPY). Waiting for data...")

    # --- Callbacks ---
    def curr_xyz_cb(self, msg): self.curr_xyz = msg
    def tgt_xyz_cb(self, msg): self.tgt_xyz = msg
    def curr_rpy_cb(self, msg): self.curr_rpy = msg
    def tgt_rpy_cb(self, msg): self.tgt_rpy = msg

    def metrics_loop(self):
        # รอให้ได้ข้อมูลครบทั้งตำแหน่งและมุม
        if None in [self.curr_xyz, self.tgt_xyz, self.curr_rpy, self.tgt_rpy]:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # 1. เช็คว่ามี Step Input สำหรับ XYZ หรือไม่
        step_x = self.tracker_x.check_step_input(self.tgt_xyz.x, self.curr_xyz.x, current_time)
        step_y = self.tracker_y.check_step_input(self.tgt_xyz.y, self.curr_xyz.y, current_time)
        step_z = self.tracker_z.check_step_input(self.tgt_xyz.z, self.curr_xyz.z, current_time)
        
        # 2. เช็คว่ามี Step Input สำหรับ RPY หรือไม่
        step_roll = self.tracker_roll.check_step_input(self.tgt_rpy.x, self.curr_rpy.x, current_time)
        step_pitch = self.tracker_pitch.check_step_input(self.tgt_rpy.y, self.curr_rpy.y, current_time)
        step_yaw = self.tracker_yaw.check_step_input(self.tgt_rpy.z, self.curr_rpy.z, current_time)
        
        if any([step_x, step_y, step_z, step_roll, step_pitch, step_yaw]):
            # พิมพ์บอกใน Terminal เล็กน้อยเพื่อความสังเกตง่าย
            pass 

        # 3. อัปเดตการคำนวณทั้งหมด
        self.tracker_x.update(self.curr_xyz.x, self.tgt_xyz.x, current_time)
        self.tracker_y.update(self.curr_xyz.y, self.tgt_xyz.y, current_time)
        self.tracker_z.update(self.curr_xyz.z, self.tgt_xyz.z, current_time)
        
        self.tracker_roll.update(self.curr_rpy.x, self.tgt_rpy.x, current_time)
        self.tracker_pitch.update(self.curr_rpy.y, self.tgt_rpy.y, current_time)
        self.tracker_yaw.update(self.curr_rpy.z, self.tgt_rpy.z, current_time)

    def print_report(self):
        if self.curr_xyz is None or self.curr_rpy is None:
            return
            
        self.get_logger().info("\n================ PERFORMANCE REPORT ================")
        
        trackers = [
            self.tracker_x, self.tracker_y, self.tracker_z,
            self.tracker_roll, self.tracker_pitch, self.tracker_yaw
        ]
        
        for tracker in trackers:
            status = "SETTLED" if tracker.is_settled else "TRACKING..." if tracker.tracking_step else "IDLE"
            
            # แสดงเฉพาะแกนที่มีการเปลี่ยนเป้าหมาย (Step) หรือมี Error สะสมอยู่ จะได้ไม่รกจอ
            if tracker.n_samples > 0:
                self.get_logger().info(f"[{tracker.name:^7}] Status: {status}")
                self.get_logger().info(f"          MSE:  {tracker.mse:.6f} | RMSE: {tracker.rmse:.4f}")
                
                if tracker.tracking_step:
                    self.get_logger().info(f"          Overshoot: {tracker.overshoot_pct:.2f}%")
                    if tracker.is_settled:
                        self.get_logger().info(f"          Settling Time: {tracker.settling_time:.2f} sec")
                        self.get_logger().info(f"          SS-Error: {tracker.ss_error:.4f}")
                    else:
                        self.get_logger().info("          Settling Time: Waiting to settle...")
        self.get_logger().info("====================================================\n")

def main(args=None):
    rclpy.init(args=args)
    node = PerformanceMetricsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()