import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped, PoseWithCovariance, Pose, Twist
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from pyproj import Proj
from tf_transformations import quaternion_from_euler
import math
from pyproj import Proj, Transformer
import numpy as np


def estimateParentGPS():
    lat_child =[57.760136587999995, 57.760137113999996,57.760137674999996 ]
    lon_child=[16.679573972,16.679563305000002,16.679556202 ]
    tf_parent2child=[(-83.12232422770467, -36.98518464714289),(-83.76454682496842, -36.946076604537666),(-84.20610611571465, -36.92797613609582) ] #(x,y,z)
    # Origin for ENU conversion
    origin_lat = lat_child[0]
    origin_lon = lon_child[0]

    # Transformer from lat/lon to local ENU (meters)
    proj_enu = Transformer.from_crs("EPSG:4326", f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +k=1 +x_0=0 +y_0=0", always_xy=True)

    parent_positions = []

    for (lat, lon), (dx, dy) in zip(zip(lat_child, lon_child), tf_parent2child):
        # Convert child GPS to ENU
        x_child, y_child = proj_enu.transform(lon, lat)
        
        # Compute parent position in ENU
        x_parent = x_child - dx
        y_parent = y_child - dy

        parent_positions.append((x_parent, y_parent))

    # Average to estimate fixed parent position
    x_avg = np.mean([p[0] for p in parent_positions])
    y_avg = np.mean([p[1] for p in parent_positions])

    # Convert back to lat/lon
    proj_ll = Transformer.from_crs(f"+proj=tmerc +lat_0={origin_lat} +lon_0={origin_lon} +k=1 +x_0=0 +y_0=0", "EPSG:4326", always_xy=True)
    lon_parent, lat_parent = proj_ll.transform(x_avg, y_avg)

    print(f"Estimated parent GPS position: lat={lat_parent}, lon={lon_parent}")
    return lon_parent, lat_parent
    
class GpsToTfAndOdom(Node):
    def __init__(self):
        super().__init__('gps_to_tf_and_odom')
        

        self.br = TransformBroadcaster(self)
        self.odom_pub = self.create_publisher(Odometry, '/gps/Odom', 10)

        self.proj = Proj(proj='utm', zone=33, ellps='WGS84')  # set your UTM zone!
        if False:
            self.origin_x = None
            self.origin_y = None
            self.parent_frame= 'map_gt' 
        else:
            odom_long,odom_lat= estimateParentGPS()
            self.origin_x, self.origin_y = self.proj(odom_long,odom_lat)
            self.parent_frame= 'odom' 
            self.last_x, self.last_y = self.proj(odom_long,odom_lat)
            
  
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)

        

    def gps_callback(self, msg: NavSatFix):
        x, y = self.proj(msg.longitude, msg.latitude)
        if self.origin_x is None:
            self.origin_x = x
            self.origin_y = y
            self.last_x = x
            self.last_y= y

        dx = x - self.origin_x
        dy = y - self.origin_y
        dz = 0.0

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self.parent_frame
        t.child_frame_id = 'Obstacle_gt'
        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = dz
        yaw = math.atan2((y-self.last_y), (x - self.last_x))
        q = quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = t.header.stamp
        odom.header.frame_id = 'map_gt'
        odom.child_frame_id = 'Obstacle_gt_temporal'
        odom.pose.pose.position.x = dx
        odom.pose.pose.position.y = dy
        odom.pose.pose.position.z = dz
        odom.pose.pose.orientation.w = 1.0  # no orientation

        # Zero velocity since no IMU/speed info
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0

        self.last_x = x
        self.last_y= y

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = GpsToTfAndOdom()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
