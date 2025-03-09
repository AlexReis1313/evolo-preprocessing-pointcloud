
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
#from tf2_ros import ExtrapolationException, ConnectivityException, LookupException
from geometry_msgs.msg import TransformStamped
import tf_transformations
import math
from rclpy.qos import QoSProfile, DurabilityPolicy
import rclpy.time



class MapToCloudTfPublisher(Node):
    def __init__(self):
        super().__init__('map_to_cloud_tf_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        #self.qos_profile = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL, depth=10) #Add QOS profile
        #self.tf_buffer = Buffer(qos=self.qos_profile) #pass the qos profile to the buffer.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.0001, self.publish_cloud_tf)

    def publish_cloud_tf(self):
        try:
            #transform_os_sensor = self.tf_buffer.lookup_transform('map', 'os_sensor', time=0)
            transform_os_sensor = self.tf_buffer.lookup_transform('map', 'os_sensor', rclpy.time.Time())

            cloud_transform = TransformStamped()
            cloud_transform.header.stamp = self.get_clock().now().to_msg()
            cloud_transform.header.frame_id = 'map'
            cloud_transform.child_frame_id = 'cloud'

            cloud_transform.transform.translation.x = transform_os_sensor.transform.translation.x
            cloud_transform.transform.translation.y = transform_os_sensor.transform.translation.y
            cloud_transform.transform.translation.z = transform_os_sensor.transform.translation.z
            """
            # Extract yaw from os_sensor's quaternion
            quaternion_os_sensor = (
                transform_os_sensor.transform.rotation.x,
                transform_os_sensor.transform.rotation.y,
                transform_os_sensor.transform.rotation.z,
                transform_os_sensor.transform.rotation.w,
            )
            _, _, yaw = tf_transformations.euler_from_quaternion(quaternion_os_sensor)

            # Create cloud's quaternion with zero roll and pitch, and the os_sensor's yaw
            cloud_quaternion = tf_transformations.quaternion_from_euler(0.0, 0.0, yaw)

            cloud_transform.transform.rotation.x = cloud_quaternion[0]
            cloud_transform.transform.rotation.y = cloud_quaternion[1]
            cloud_transform.transform.rotation.z = cloud_quaternion[2]
            cloud_transform.transform.rotation.w = cloud_quaternion[3]
            """
            cloud_transform.transform.rotation.x = 0.0
            cloud_transform.transform.rotation.y = 0.0
            cloud_transform.transform.rotation.z = 0.0
            cloud_transform.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(cloud_transform)

        except Exception as e:
            self.get_logger().warn(f"Could not transform: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    map_to_cloud_tf_publisher = MapToCloudTfPublisher()
    rclpy.spin(map_to_cloud_tf_publisher)
    map_to_cloud_tf_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()