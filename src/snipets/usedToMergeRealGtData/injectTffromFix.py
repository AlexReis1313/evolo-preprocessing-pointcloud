import os
import sys
import rclpy
import rosbag2_py
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import TransformStamped
from tf2_msgs.msg import TFMessage
from builtin_interfaces.msg import Time


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
    
class GpsToTfAndOdom():
    def __init__(self):
        


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
        

        

    def generate_tf_from_fix(self, fix_msg):
        x, y = self.proj(fix_msg.longitude, fix_msg.latitude)
        if self.origin_x is None:
            self.origin_x = x
            self.origin_y = y
            self.last_x = x
            self.last_y= y

        dx = x - self.origin_x
        dy = y - self.origin_y
        dz = 0.0

        t = TransformStamped()
        t.header.stamp = fix_msg.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "Obstacle_Gt"

        t.transform.translation.x = dx
        t.transform.translation.y = dy
        t.transform.translation.z = dz
        yaw = math.atan2((y-self.last_y), (x - self.last_x))
        q = quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]




        return t
        




"""
def generate_tf_from_fix(fix_msg):
  
  
    t = TransformStamped()
    t.header.stamp = fix_msg.header.stamp
    t.header.frame_id = "odom"
    t.child_frame_id = "Obstacle_Gt"

    # Dummy transform from latitude and longitude (replace this with real logic)
    t.transform.translation.x = fix_msg.latitude
    t.transform.translation.y = fix_msg.longitude
    t.transform.translation.z = fix_msg.altitude

    # Identity rotation (no rotation)
    t.transform.rotation.w = 1.0
    t.transform.rotation.x = 0.0
    t.transform.rotation.y = 0.0
    t.transform.rotation.z = 0.0

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

    return t
"""

def inject_tf_from_fix(input_bag_path, output_bag_path):
    #os.makedirs(output_bag_path, exist_ok=True)

    # Initialize reader
    reader = rosbag2_py.SequentialReader()
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader.open(storage_options, converter_options)

    # Get topic types
    topic_types = reader.get_all_topics_and_types()
    type_map = {topic.name: topic.type for topic in topic_types}

    msg_type_map = {name: get_message(type_name) for name, type_name in type_map.items()}

    """
    msg_type_map = {}
    for name, type_name in type_map.items():
        try:
            msg_type_map[name] = get_message(type_name)
        except (AttributeError, ModuleNotFoundError, ValueError) as e:
            print(f"[INFO] Skipping deserialization for unknown message type '{type_name}' on topic '{name}'. It will be copied raw.")
            msg_type_map[name] = None  # Mark as unknown
            """
    # Initialize writer
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )

    # Register original topics
    for topic in topic_types:
        writer.create_topic(rosbag2_py.TopicMetadata(
            name=topic.name,
            type=topic.type,
            serialization_format="cdr"
        ))

    # Register /tf topic
    #writer.create_topic(rosbag2_py.TopicMetadata(
    #    name="/tf",
    #    type="tf2_msgs/msg/TFMessage",
    #    serialization_format="cdr"
    #))

    # Iterate through input bag and process

    converter = GpsToTfAndOdom()
    while reader.has_next():
        topic_name, data, timestamp = reader.read_next()
        msg_type = msg_type_map[topic_name]
        """
        if msg_type is None:
            # Unknown/custom type — just copy raw message
            writer.write(topic_name, data, timestamp)
            continue
        """
        msg = deserialize_message(data, msg_type)

        # Write original message
        writer.write(topic_name, data, timestamp)

        # If it's a /fix message, generate and write a /tf message
        if topic_name == "/fix":
            tf_transform = converter.generate_tf_from_fix(msg)
            tf_msg = TFMessage(transforms=[tf_transform])
            serialized_tf = serialize_message(tf_msg)
            writer.write("/tf", serialized_tf, timestamp)

    print(f"Output bag written to {output_bag_path}")

# CLI
if __name__ == "__main__":
    #if len(sys.argv) != 3:
    #    print("Usage: python inject_tf_from_fix.py <input_bag> <output_bag>")
    #    sys.exit(1)

    input_bag = '/media/alex/KINGSTON/waraps/busterGT/completeBusterFix'
    
    output_bag = '/media/alex/KINGSTON/waraps/busterGT/completeBusterTf'

    rclpy.init()
    inject_tf_from_fix(input_bag, output_bag)
    rclpy.shutdown()
