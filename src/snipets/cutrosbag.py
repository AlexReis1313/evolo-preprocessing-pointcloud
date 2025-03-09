#initial time 1741118647 - 0
#initial move 1741118840 - 193
#final time 1741119321 - 674

import rosbag2_py
from rclpy.time import Time

input_bag_path = "/media/alex/KINGSTON/2024-11-27-baggensfjarden/2.marina/ros2bags/marinabag/marinabag_0.db3"
output_bag_path = "/media/alex/KINGSTON/2024-11-27-baggensfjarden/2.marina/ros2bags/marinabag/marinabag_0short.db3"
trim_time_seconds = 193.0
end_time = 674

# Open old bag for reading
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=input_bag_path, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))

# Open new bag for writing
writer = rosbag2_py.SequentialWriter()
writer.open(rosbag2_py.StorageOptions(uri=output_bag_path, storage_id="sqlite3"),
            rosbag2_py.ConverterOptions("", ""))

# Get starting timestamp
metadata = reader.get_metadata()
start_time_ns = metadata.starting_time.nanoseconds
trim_time_ns = start_time_ns + int(trim_time_seconds * 1e9)
end_time_ns = start_time_ns + int(end_time * 1e9)
# Register topics in new bag before writing
for topic_metadata in metadata.topics_with_message_count:
    writer.create_topic(topic_metadata.topic_metadata)
# Process messages
while reader.has_next():
    topic, msg, t = reader.read_next()  # 't' is already in nanoseconds (int)
    if (t >= trim_time_ns) and(t <= end_time_ns):
        writer.write(topic, msg, t)
