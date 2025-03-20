#!/usr/bin/env python

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2


class PointCloudToOccupancyGrid:
    def __init__(self):
        rospy.init_node("point_cloud_to_occupancy_grid", anonymous=True)
        self.pointcloud_sub = rospy.Subscriber(
            "/orb_slam3/all_points", PointCloud2, self.pointcloud_callback
        )
        self.map_pub = rospy.Publisher("/map", OccupancyGrid, queue_size=10)

        # Parameters
        self.resolution = 0.05  # meters per pixel
        self.height_threshold = 2.0  # threshold for filtering points by height
        self.map_size = 10.0  # map size in meters
        self.map_width = int(self.map_size / self.resolution)  # in pixels
        self.map_height = int(self.map_size / self.resolution)  # in pixels
        self.origin_x = -self.map_size / 2.0  # origin of the map in meters
        self.origin_z = -self.map_size / 2.0  # origin of the map in meters

        self.occupancy_grid = None
        # Timer for publishing the map
        rospy.Timer(rospy.Duration(0.1), self.publish_map)

    def pointcloud_callback(self, data):
        # Initialize occupancy grid map
        occupancy_grid = OccupancyGrid()
        occupancy_grid.header = data.header
        occupancy_grid.info.width = self.map_width
        occupancy_grid.info.height = self.map_height
        occupancy_grid.info.resolution = self.resolution
        occupancy_grid.info.origin.position.x = self.origin_x
        occupancy_grid.info.origin.position.y = 0  # Y-axis is not relevant
        occupancy_grid.info.origin.position.z = self.origin_z

        # Initialize map data
        occupancy_grid.data = [-1] * (self.map_width * self.map_height)

        # Process point cloud data
        for point in pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=True):
            if point[1] < self.height_threshold:
                grid_x = int((point[0] - self.origin_x) / self.resolution)
                grid_y = int((point[2] - self.origin_z) / self.resolution)
                if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                    index = grid_y * self.map_width + grid_x
                    occupancy_grid.data[index] = 100  # Mark cell as occupied

        self.occupancy_grid = occupancy_grid

    def publish_map(self, event):
        if rospy.is_shutdown():
            return
        if self.occupancy_grid is None:
            return
        # Publish the occupancy grid map
        self.occupancy_grid.header.stamp = rospy.Time.now()
        self.map_pub.publish(self.occupancy_grid)


if __name__ == "__main__":
    try:
        pc_to_og = PointCloudToOccupancyGrid()
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        pass
