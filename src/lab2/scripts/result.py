#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Vector3
from std_msgs.msg import String
import math

class AxisTracker:
    def __init__(self, name, settle_threshold, step_threshold, settle_time_req=0.5):
        self.name = name
        self.settle_threshold = settle_threshold
        self.step_threshold = step_threshold
        self.settle_time_req = settle_time_req
        self.reset() # เรียกใช้ reset ตอนสร้าง object เลย

    def reset(self):
        """ฟังก์ชันสำหรับล้างข้อมูลเก่าทั้งหมด เพื่อเริ่มการทดลองรอบใหม่"""
        self.n_samples = 0
        self.sum_sq_err = 0.0
        self.rmse = 0.0
        self.mse = 0.0
        
        self.tracking_step = False
        self.start_val = 0.0
        self.target_val = 0.0
        self.step_start_time = 0.0
        self.max_val_reached = 0.0
        self.overshoot_pct = 0.0
        
        self.time_entered_settle = None
        self.is_settled = False
        self.settling_time = 0.0
        self.ss_error = 0.0

    def check_step_input(self, current_target, current_pos, current_time):
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
        
        # --- Subscribers ---
        self.sub_curr_xyz = self.create_subscription(Vector3, '/debug/current_xyz', self.curr_xyz_cb, 10)
        self.sub_tgt_xyz = self.create_subscription(Vector3, '/debug/target_xyz', self.tgt_xyz_cb, 10)
        self.sub_curr_rpy = self.create_subscription(Vector3, '/debug/current_rpy', self.curr_rpy_cb, 10)
        self.sub_tgt_rpy = self.create_subscription(Vector3, '/debug/target_rpy', self.tgt_rpy_cb, 10)
        
        # !! เพิ่ม Subscriber สำหรับรับค่า Mode !!
        self.mode_sub = self.create_subscription(String, '/set_flight_mode', self.mode_callback, 10)

        # --- Trackers ---
        self.tracker_x = AxisTracker('X-Pos', settle_threshold=0.05, step_threshold=0.1)
        self.tracker_y = AxisTracker('Y-Pos', settle_threshold=0.05, step_threshold=0.1)
        self.tracker_z = AxisTracker('Z-Pos', settle_threshold=0.05, step_threshold=0.1)
        self.tracker_roll = AxisTracker('Roll', settle_threshold=0.035, step_threshold=0.05)
        self.tracker_pitch = AxisTracker('Pitch', settle_threshold=0.035, step_threshold=0.05)
        self.tracker_yaw = AxisTracker('Yaw', settle_threshold=0.035, step_threshold=0.05)

        self.trackers = [
            self.tracker_x, self.tracker_y, self.tracker_z,
            self.tracker_roll, self.tracker_pitch, self.tracker_yaw
        ]

        self.curr_xyz = None; self.tgt_xyz = None
        self.curr_rpy = None; self.tgt_rpy = None
        
        # ==========================================
        # การตั้งค่าเงื่อนไขการทดลอง (Experiment Setup)
        # ==========================================
        self.is_evaluating = False           # สถานะกำลังเก็บผลหรือไม่
        self.eval_duration = 40.0            # **ตั้งเวลาเก็บผลการทดลอง (วินาที) เช่น 30 วินาที**
        self.eval_start_time = 0.0
        self.current_flight_mode = "NONE"

        # Timer
        self.timer = self.create_timer(0.02, self.metrics_loop) # 50Hz
        self.report_timer = self.create_timer(2.0, self.print_periodic_report)
        
        self.get_logger().info(f"Metrics Node Ready. Waiting for Flight Mode to start a {self.eval_duration}s evaluation...")

    # --- Callbacks ---
    def curr_xyz_cb(self, msg): self.curr_xyz = msg
    def tgt_xyz_cb(self, msg): self.tgt_xyz = msg
    def curr_rpy_cb(self, msg): self.curr_rpy = msg
    def tgt_rpy_cb(self, msg): self.tgt_rpy = msg

    def mode_callback(self, msg):
        new_mode = msg.data.upper()
        if new_mode == "IDLE":
            self.is_evaluating = False
            self.get_logger().info("Evaluation STOPPED because mode is IDLE.")
            return
            
        # ถ้าได้รับ Mode ใหม่ที่ไม่ใช่ IDLE ให้เริ่มการทดลองใหม่
        if new_mode != self.current_flight_mode:
            self.current_flight_mode = new_mode
            self.get_logger().info(f"\n=============================================")
            self.get_logger().info(f"🚀 STARTING NEW EVALUATION FOR MODE: {new_mode}")
            self.get_logger().info(f"⏱️ Duration: {self.eval_duration} seconds")
            self.get_logger().info(f"=============================================\n")
            
            # ล้างค่าเก่าทิ้งให้หมด
            for tracker in self.trackers:
                tracker.reset()
                
            self.eval_start_time = self.get_clock().now().nanoseconds / 1e9
            self.is_evaluating = True

    def metrics_loop(self):
        # ถ้าไม่ได้อยู่ในสถานะประเมินผล ให้ข้ามการคำนวณไปเลย
        if not self.is_evaluating:
            return
            
        if None in [self.curr_xyz, self.tgt_xyz, self.curr_rpy, self.tgt_rpy]:
            return
            
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # เช็คว่าหมดเวลาทดลองหรือยัง?
        elapsed_time = current_time - self.eval_start_time
        if elapsed_time >= self.eval_duration:
            self.is_evaluating = False # หยุดบันทึก
            self.print_final_report()  # พิมพ์ผลสรุปสุดท้าย
            return
        
        # 1. เช็ค Step Input
        self.tracker_x.check_step_input(self.tgt_xyz.x, self.curr_xyz.x, current_time)
        self.tracker_y.check_step_input(self.tgt_xyz.y, self.curr_xyz.y, current_time)
        self.tracker_z.check_step_input(self.tgt_xyz.z, self.curr_xyz.z, current_time)
        self.tracker_roll.check_step_input(self.tgt_rpy.x, self.curr_rpy.x, current_time)
        self.tracker_pitch.check_step_input(self.tgt_rpy.y, self.curr_rpy.y, current_time)
        self.tracker_yaw.check_step_input(self.tgt_rpy.z, self.curr_rpy.z, current_time)

        # 2. อัปเดตการคำนวณ
        self.tracker_x.update(self.curr_xyz.x, self.tgt_xyz.x, current_time)
        self.tracker_y.update(self.curr_xyz.y, self.tgt_xyz.y, current_time)
        self.tracker_z.update(self.curr_xyz.z, self.tgt_xyz.z, current_time)
        self.tracker_roll.update(self.curr_rpy.x, self.tgt_rpy.x, current_time)
        self.tracker_pitch.update(self.curr_rpy.y, self.tgt_rpy.y, current_time)
        self.tracker_yaw.update(self.curr_rpy.z, self.tgt_rpy.z, current_time)

    def print_periodic_report(self):
        # ปรินต์อัปเดตสั้นๆ ระหว่างรัน
        if not self.is_evaluating:
            return
        current_time = self.get_clock().now().nanoseconds / 1e9
        elapsed = current_time - self.eval_start_time
        self.get_logger().info(f"⏳ Evaluating {self.current_flight_mode}... [{elapsed:.1f}/{self.eval_duration:.1f} sec]")

    def print_final_report(self):
        # ปรินต์สรุปแบบเต็มเมื่อจบการทดลอง
        self.get_logger().info("\n" + "="*50)
        self.get_logger().info(f" 🎯 FINAL REPORT: {self.current_flight_mode}")
        self.get_logger().info("="*50)
        
        for tracker in self.trackers:
            if tracker.n_samples > 0:
                self.get_logger().info(f"[{tracker.name:^7}] Samples: {tracker.n_samples}")
                self.get_logger().info(f"          MSE:  {tracker.mse:.6f} | RMSE: {tracker.rmse:.4f}")
                
                if tracker.tracking_step:
                    self.get_logger().info(f"          Overshoot: {tracker.overshoot_pct:.2f}%")
                    if tracker.is_settled:
                        self.get_logger().info(f"          Settling Time: {tracker.settling_time:.2f} sec")
                        self.get_logger().info(f"          SS-Error: {tracker.ss_error:.4f}")
                    else:
                        self.get_logger().info("          Settling Time: FAILED to settle within evaluation time.")
                self.get_logger().info("-" * 40)
        self.get_logger().info("✅ Evaluation Completed. Waiting for new mode...\n")

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