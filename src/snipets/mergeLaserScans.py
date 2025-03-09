import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy


class MinRangeLaserScanMerger(Node):
    def __init__(self):
        super().__init__('min_range_laser_scan_merger')
        best_effort_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)

        self.scan1_sub = self.create_subscription(
            LaserScan,
            '/scanner/scan/short',
            self.scan1_callback,
            best_effort_qos
        )
        self.scan2_sub = self.create_subscription(
            LaserScan,
            '/scanner/scan/long',
            self.scan2_callback,
            best_effort_qos
        )
        self.merged_scan_pub = self.create_publisher(
            LaserScan,
            '/scanner/scan/merged_scan',
            10
        )

        self.scan1_data = None
        self.scan2_data = None

    def scan1_callback(self, msg):
        self.scan1_data = msg
        self.merge_and_publish()

    def scan2_callback(self, msg):
        self.scan2_data = msg
        self.merge_and_publish()

    def merge_and_publish(self):
        if self.scan1_data is None or self.scan2_data is None:
            return
        else:
            scan1 = self.scan1_data
            scan2 = self.scan2_data

            if scan1.header.frame_id != scan2.header.frame_id:
                self.get_logger().warn("LaserScan frame IDs do not match!")
                return

            if len(scan1.ranges) != len(scan2.ranges):
                self.get_logger().warn("LaserScan range lengths do not match!")
                return

            merged_scan = LaserScan()
            merged_scan.header.stamp = self.get_clock().now().to_msg()
            merged_scan.header.frame_id = scan1.header.frame_id
            merged_scan.angle_min = scan1.angle_min
            merged_scan.angle_max = scan1.angle_max
            merged_scan.angle_increment = scan1.angle_increment
            merged_scan.time_increment = scan1.time_increment
            merged_scan.scan_time = scan1.scan_time
            merged_scan.range_min = min(scan1.range_min, scan2.range_min)
            merged_scan.range_max = max(scan1.range_max, scan2.range_max) #This is changed from max to min.
            merged_scan.ranges = []
            merged_scan.intensities = [] # if you need intensities, merge them as well.

            for i in range(len(scan1.ranges)):
                merged_scan.ranges.append(min(scan1.ranges[i], scan2.ranges[i]))

            self.merged_scan_pub.publish(merged_scan)
            self.scan1_data = None
            self.scan2_data = None

def main(args=None):
    rclpy.init(args=args)
    min_range_merger = MinRangeLaserScanMerger()
    rclpy.spin(min_range_merger)
    min_range_merger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()