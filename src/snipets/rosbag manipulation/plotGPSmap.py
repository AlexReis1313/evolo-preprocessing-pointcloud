import rclpy
import sys
import rosbag2_py
from sensor_msgs.msg import NavSatFix
from rclpy.serialization import deserialize_message
import folium

def read_gps_from_bag(bag_path, topic='/fix'):
    storage_options = rosbag2_py.StorageOptions(uri=bag_path, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions('', '')
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_dict = {t.name: t.type for t in topic_types}

    gps_points = []

    while reader.has_next():
        (topic_name, data, timestamp) = reader.read_next()
        if topic_name == topic:
            msg_type = NavSatFix
            msg = deserialize_message(data, msg_type)
            gps_points.append((msg.latitude, msg.longitude))

    return gps_points


def create_map(gps_points, output_html='gps_map.html'):
    if not gps_points:
        print("No GPS data found.")
        return

    # Center map on first point
    m = folium.Map(location=gps_points[0], zoom_start=17)
    folium.PolyLine(gps_points, color="blue", weight=3).add_to(m)
    for i, point in enumerate(gps_points):
        folium.CircleMarker(location=point, radius=1, color='red').add_to(m)

    m.save(output_html)
    print(f"Map saved to {output_html}")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 plot_gps_from_bag.py /path/to/rosbag2")
        exit(1)

    bag_path = sys.argv[1]
    gps_data = read_gps_from_bag(bag_path)
    create_map(gps_data)

# to run:
# python3 plot_gps_from_bag.py ~/your_rosbag_folder
