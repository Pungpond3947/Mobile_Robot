#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
from nav_msgs.msg import Path
from tf2_ros import TransformBroadcaster
import numpy as np

class TurtlebotPoseNode(Node):
    def __init__(self):
        super().__init__('TF_node')
        
        self.create_subscription(PoseStamped, "/turtle_pose_wheel", self.wheel_pose_callback, 10)
        self.create_subscription(PoseStamped, "/turtle_pose_EKF", self.ekf_pose_callback, 10)
        self.create_subscription(PoseStamped, "/turtle_pose_ICP", self.icp_pose_callback, 10)
        
        self.wheel_path_pub = self.create_publisher(Path, "/path_wheel", 10)
        self.ekf_path_pub = self.create_publisher(Path, "/path_ekf", 10)
        self.icp_path_pub = self.create_publisher(Path, "/path_icp", 10)
        
        self.tf_bct = TransformBroadcaster(self)
        
        self.wheel_path_msg = Path()
        self.wheel_path_msg.header.frame_id = "odom"

        self.ekf_path_msg = Path()
        self.ekf_path_msg.header.frame_id = "odom"
        
        self.icp_path_msg = Path()
        self.icp_path_msg.header.frame_id = "odom"

    def ekf_pose_callback(self, msg):
        current_time = msg.header.stamp
        self.broadcast_and_publish(msg.pose, current_time, "base_link_ekf", self.ekf_path_msg, self.ekf_path_pub)

    def icp_pose_callback(self, msg):
        current_time = msg.header.stamp 
        self.broadcast_and_publish(msg.pose, current_time, "base_link_icp", self.icp_path_msg, self.icp_path_pub)

    def wheel_pose_callback(self, msg):
        current_time = msg.header.stamp
        self.broadcast_and_publish(msg.pose, current_time, "base_link_wheel", self.wheel_path_msg, self.wheel_path_pub)

    def broadcast_and_publish(self, pose_msg, time, frame_id, path_msg, path_pub):
        t = TransformStamped()
        t.header.stamp = time
        t.header.frame_id = "odom"
        t.child_frame_id = frame_id
        t.transform.translation.x = pose_msg.position.x
        t.transform.translation.y = pose_msg.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = pose_msg.orientation
        self.tf_bct.sendTransform(t)
        
        # ส่ง Path
        ps = PoseStamped()
        ps.header.stamp = time
        ps.header.frame_id = "odom"
        ps.pose = pose_msg
        path_msg.poses.append(ps)
        path_msg.header.stamp = time
        path_pub.publish(path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TurtlebotPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__=='__main__':
    main()