The map to view on the simulator is a sensor_msgs.msg::PointCloud2 on the topic /Evolo/mqtt/map.
Each point in the PointCloud corresponds to the center of a 1x1x1m occupied voxel (or 1x1m occupied cell).

/Evolo/mqtt/map is published at 2Hz and is given in the Evolo base_footprint frame.
base_footprint has equal x,y,yaw to Evolo base_link - everything else is zero.
