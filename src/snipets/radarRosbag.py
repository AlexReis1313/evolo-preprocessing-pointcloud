import rclpy
from rclpy.serialization import serialize_message
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from rosbag2_py._storage import TopicMetadata

import rosbag2_py
import csv
import time
import os

CSV_PATH = 'Pcl.csv'   # << Replace with your CSV path
BAG_PATH = 'Radar1swaterPointcloud_bag'             # Bag will be created here
TOPIC_NAME = '/radar/pointcloud'
FRAME_ID = 'radar_link'
FRAME_INTERVAL = 1.0 / 15.0              # 15 FPS (66.7ms between frames)

def create_pointcloud2(points, stamp, frame_id=FRAME_ID):
    header = Header()
    header.stamp = stamp
    header.frame_id = frame_id

    fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    return point_cloud2.create_cloud(header, fields, points)

def main():
    rclpy.init()
    node = rclpy.create_node('pointcloud_to_bag_writer')

    # Ensure output directory exists and is empty
    if os.path.exists(BAG_PATH):
        print(f"Error: Bag path '{BAG_PATH}' already exists. Please remove it or change BAG_PATH.")
        return

    # Set up rosbag2 writer
    storage_options = rosbag2_py.StorageOptions(uri=BAG_PATH, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(input_serialization_format='cdr', output_serialization_format='cdr')


    writer = rosbag2_py.SequentialWriter()
    writer.open(storage_options, converter_options)


    writer.create_topic(TopicMetadata(
        name=TOPIC_NAME,
        type='sensor_msgs/msg/PointCloud2',
        serialization_format='cdr'
    ))

    # Open and parse CSV
    with open(CSV_PATH, 'r') as f:
        reader = csv.reader(f)
        next(reader)  # Skip header
        next(reader) # Skip header
        next(reader) # Skip header


        buffered_line = None
        clock = node.get_clock()
        now = clock.now()

        while True:
            points = []
            frame_number = None

            if buffered_line:
                row = buffered_line
                buffered_line = None
            else:
                try:
                    row = next(reader)
                except StopIteration:
                    break

            frame_number = int(row[0])

            # Process lines for current frame
            while True:
                xpos = float(row[14])
                ypos = float(row[15])
                zpos = float(row[16])
                intensity = float(row[12])
                points.append([xpos, ypos, zpos, intensity])

                try:
                    row = next(reader)
                    next_frame_number = int(row[0])
                    if next_frame_number != frame_number:
                        buffered_line = row
                        break
                except StopIteration:
                    break

            # Simulate 15Hz timestamp progression
            now = rclpy.time.Time(seconds=now.nanoseconds / 1e9 + FRAME_INTERVAL)

            # Create and write message
            pc2_msg = create_pointcloud2(points, now.to_msg())
            writer.write(TOPIC_NAME, serialize_message(pc2_msg), now.nanoseconds)

            print(f"Wrote frame {frame_number} with {len(points)} points")

    print(f"✅ Bag written to: {BAG_PATH}")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
