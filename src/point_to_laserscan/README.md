# ROS 2 pointcloud <-> laserscan converters

This is a ROS 2 package that was addapted from the original `pointcloud_to_laserscan`, which can be found at:https://github.com/ros-perception/pointcloud_to_laserscan.

The logic to transform a Point Cloud into a LaserScan was kept the same.
Several new featrues were added:
* Transforming the initial PointCloud into a frame that maintains the same center coordinate but is 2D - zeroing Z, pitch and roll values (with relation to a given map frame). This is usefull, as it enables one to filter out points based on their coordnates relative to a fixed frame with no attitude values.
* Filter the PointCloud. Filtering methods are ment to delete noisy points and points the bellong to water reflection in a maritime surface environment. This is done using the following methods: Removal of any points that have low intensity and are bellow the water line (reflections in the water); Filtering any points that do not pass a threshold of number of neighbouring points for a given radius (deletes noisy points - this radius threshold increses linearly with the distasnce to the center of the PointCloud, in order to account for LiDAR Ray sparisty with distance increase).
* LaserScan is constructed from points in only specified regions of the PointCloud. This is done via the specification of 2 range regions - For low range, the parameters of points to keep are: Minimum range, minimum z/height, maximum z/height, Maximum Range/ transition range. For the high range section, the parameters are: Minimum range/ transition range, minimum z/height, maximum z/height, Maximum Range. Final LaserScan is build from merging these 2 LaserScans into 1 LaserScan with, at most, 1 point per angle value.

