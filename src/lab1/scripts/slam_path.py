#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import tf2_ros
import math

class SlamPathNode(Node):
    def __init__(self):
        super().__init__('slam_path_node')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.slam_path_pub = self.create_publisher(Path, "/path_slam", 10)
        self.slam_path_msg = Path()
        self.slam_path_msg.header.frame_id = "map"

        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_link_icp',
                rclpy.time.Time()
            )
            
            ps = PoseStamped()
            ps.header.stamp = self.get_clock().now().to_msg()
            ps.header.frame_id = "map"
            ps.pose.position.x = t.transform.translation.x
            ps.pose.position.y = t.transform.translation.y
            ps.pose.orientation = t.transform.rotation
            
            self.slam_path_msg.poses.append(ps)
            self.slam_path_msg.header.stamp = ps.header.stamp
            self.slam_path_pub.publish(self.slam_path_msg)
            
        except tf2_ros.TransformException as ex:
            pass

def main(args=None):
    rclpy.init(args=args)
    node = SlamPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__=='__main__':
    main()