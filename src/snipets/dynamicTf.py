#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import time

class DynamicTFPublisher(Node):
    def __init__(self):
        
        super().__init__('dynamic_tf_publisher')
        print("Please do: ros2 param get /dynamic_tf_publisher 'use_sim_time'")
        self.broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.t = 0.0

    def timer_callback(self):
        now = self.get_clock().now().to_msg()
        tf_msg = TransformStamped()
        tf_msg.header.stamp = now
        tf_msg.header.frame_id = 'map'
        tf_msg.child_frame_id = 'odom'
        tf_msg.transform.translation.x = 0.0;#2.0 + 0.5 * rclpy.time.Time(seconds=self.t).nanoseconds / 1e9  # Simulate motion
        tf_msg.transform.translation.y = 0.0
        tf_msg.transform.translation.z = 0.0
        tf_msg.transform.rotation.x = 0.0
        tf_msg.transform.rotation.y = 0.0
        tf_msg.transform.rotation.z = 0.0
        tf_msg.transform.rotation.w = 1.0
        self.broadcaster.sendTransform(tf_msg)
        self.t += 0.1

def main():
    rclpy.init()
    node = DynamicTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
