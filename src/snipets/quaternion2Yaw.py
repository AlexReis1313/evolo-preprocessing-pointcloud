#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion
from std_msgs.msg import Float64
import math

class YawPublisher(Node):
    def __init__(self):
        super().__init__('yaw_publisher')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(Float64, 'yaw_angle', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # 10 Hz

        # Change these to your frames
        self.target_frame = 'base_link'
        self.source_frame = 'odom'

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.source_frame,
                self.target_frame,
                now
            )

            q = trans.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]
            _, _, yaw = euler_from_quaternion(quaternion)

            yaw_msg = Float64()
            yaw_msg.data = yaw
            self.publisher.publish(yaw_msg)
            self.get_logger().info(f'Published yaw: {math.degrees(yaw):.2f} deg')

        except Exception as e:
            self.get_logger().warn(f'Could not transform {self.source_frame} to {self.target_frame}: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YawPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
