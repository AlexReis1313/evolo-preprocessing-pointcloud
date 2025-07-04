#initial time 1741118647 - 0
#initial move 1741118840 - 193
#final time 1741119321 - 674

import rosbag2_py
from rclpy.time import Time

input_bag_path = "/media/alex/KINGSTON/MetricBags/completeBusterFix_andLIDAR"
output_bag_path = "/media/alex/KINGSTON/MetricBags/short_BusterGT_1"
if False:
    trim_time_seconds = 193.0
    end_time = 674
    trim_start_ns =None
    trim_end_ns = None
    #bag2
    start_sec = 1747830519
    start_nsec = 993835349
    end_sec = 1747830603
    end_nsec = 994280456
else:
    #bag 1
    sec=1747829882
    nsec=59888872
    start_sec = 1747830126
    start_nsec = 2982767
    end_sec = 1747830280 
    end_nsec = 995201385
    #bag2
    #start_sec = 1747830519
    #start_nsec = 993835349
    #end_sec = 1747830603
    #end_nsec = 994280456

    trim_start_ns = start_sec * 10**9 + start_nsec
    trim_end_ns = end_sec * 10**9 + end_nsec
    begintime = sec* 10**9 + nsec
    print('trim_start_ns',trim_start_ns, '...', trim_start_ns-begintime, '   ', start_sec - sec)
    print('trim_end_ns',trim_end_ns, '...', trim_end_ns-begintime, '   ', end_sec - sec)
    print('begintime',begintime)

    print('\n Bag 2 \n')
    start_sec = 1747830519
    start_nsec = 993835349
    end_sec = 1747830603
    end_nsec = 994280456

    trim_start_ns = start_sec * 10**9 + start_nsec
    trim_end_ns = end_sec * 10**9 + end_nsec
    begintime = sec* 10**9 + nsec
    print('trim_start_ns',trim_start_ns, '...', trim_start_ns-begintime, '   ', start_sec - sec)
    print('trim_end_ns',trim_end_ns, '...', trim_end_ns-begintime, '   ', end_sec - sec)
    print('begintime',begintime)

    


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
if trim_start_ns == None:
    start_time_ns = metadata.starting_time.nanoseconds
    trim_start_ns = start_time_ns + int(trim_time_seconds * 1e9)
    trim_end_ns = start_time_ns + int(end_time * 1e9)
# Register topics in new bag before writing
for topic_metadata in metadata.topics_with_message_count:
    writer.create_topic(topic_metadata.topic_metadata)
# Process messages
while reader.has_next():
    topic, msg, t = reader.read_next()  # 't' is already in nanoseconds (int)
    if (t >= trim_start_ns) and(t <= trim_end_ns):
        writer.write(topic, msg, t)
