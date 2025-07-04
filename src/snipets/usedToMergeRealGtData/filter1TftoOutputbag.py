import os
import sys
import rclpy
import rosbag2_py
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message

def filter_tf_to_tf_filtered(input_bag_path, output_bag_path, parent_frame, child_frame):
    # Ensure output directory exists
    #os.makedirs(output_bag_path, exist_ok=True)

    # Set up reader
    storage_options = rosbag2_py.StorageOptions(uri=input_bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    topic_types = reader.get_all_topics_and_types()

    type_map = {t.name: t.type for t in topic_types}
    if "/tf" not in type_map:
        print("Error: /tf topic not found in input bag.")
        return

    msg_type = get_message(type_map["/tf"])

    # Set up writer
    writer = rosbag2_py.SequentialWriter()
    writer.open(
        rosbag2_py.StorageOptions(uri=output_bag_path, storage_id='sqlite3'),
        rosbag2_py.ConverterOptions('', '')
    )
    writer.create_topic(rosbag2_py.TopicMetadata(
        name="/tf",
        type="tf2_msgs/msg/TFMessage",
        serialization_format="cdr"
    ))

    # Read and filter
    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic != "/tf":
            continue

        tf_msg = deserialize_message(data, msg_type)
        filtered_transforms = [
            t for t in tf_msg.transforms
            if t.header.frame_id == parent_frame and t.child_frame_id == child_frame
        ]

        if filtered_transforms:
            filtered_msg = TFMessage(transforms=filtered_transforms)
            serialized = serialize_message(filtered_msg)
            writer.write("/tf", serialized, timestamp)

    print(f"Output written to: {output_bag_path}")

# CLI usage
if __name__ == "__main__":


    input_bag = '/media/alex/KINGSTON/waraps/finalBag/allTfsGt'
    output_bag = '/media/alex/KINGSTON/waraps/finalBag/onlyGoodTf'
    parent = "odom"
    child = "Obstacle_gt"

    rclpy.init()
    filter_tf_to_tf_filtered(input_bag, output_bag, parent, child)
    rclpy.shutdown()