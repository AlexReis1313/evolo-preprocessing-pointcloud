import rclpy
from sensor_msgs.msg import NavSatFix
from rclpy.node import Node
import matplotlib.pyplot as plt
from mpl_toolkits.basemap import Basemap
import osmnx as ox

class LiveGpsPlotter(Node):
    def __init__(self):
        super().__init__('live_gps_plotter')
        self.subscription = self.create_subscription(
            NavSatFix,
            '/fix',
            self.gps_callback,
            10)

        # Store GPS coordinates for plotting
        self.gps_data = []
        self.fig, self.ax = plt.subplots(figsize=(8, 8))
        self.plot = None
        self.center = (0, 0)

        plt.ion()  # Enable interactive mode
        plt.show()

    def gps_callback(self, msg: NavSatFix):
        # Append new GPS data
        lat, lon = msg.latitude, msg.longitude
        self.gps_data.append((lat, lon))

        # Update the map if it's the first GPS data or the location changes significantly
        if len(self.gps_data) == 1 or self.should_update_map(lat, lon):
            self.center = (lat, lon)
            self.update_map(lat, lon)

        # Plot the GPS points
        self.plot_gps_points()

    def should_update_map(self, lat, lon):
        # Simple check to update map when position is far enough from previous center
        prev_lat, prev_lon = self.center
        distance = self.calculate_distance(prev_lat, prev_lon, lat, lon)
        return distance > 0.1  # Trigger map update if more than 100m apart

    def update_map(self, lat, lon):
        # Fetch a new OpenStreetMap tile centered at the current GPS position
        # Using osmnx to get a graph from the OSM database
        G = ox.graph_from_point((lat, lon), dist=1000, network_type='all')
        fig, ax = ox.plot_graph(G, show=False, close=False)

        # Update the current map
        self.ax.clear()
        ax.plot()
        self.ax.set_xlim(lon - 0.01, lon + 0.01)
        self.ax.set_ylim(lat - 0.01, lat + 0.01)
        self.fig.canvas.draw()

    def plot_gps_points(self):
        if self.plot is None:
            self.plot, = self.ax.plot([], [], 'ro', markersize=3)  # red dot for GPS points
        lats, lons = zip(*self.gps_data)
        self.plot.set_data(lons, lats)
        self.fig.canvas.draw()

    def calculate_distance(self, lat1, lon1, lat2, lon2):
        # Haversine formula to calculate distance between two GPS coordinates
        from math import radians, sin, cos, sqrt, atan2

        R = 6371.0  # Earth radius in kilometers
        lat1, lon1, lat2, lon2 = map(radians, [lat1, lon1, lat2, lon2])

        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = sin(dlat / 2)**2 + cos(lat1) * cos(lat2) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = R * c  # result in kilometers
        return distance

def main(args=None):
    rclpy.init(args=args)
    node = LiveGpsPlotter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
