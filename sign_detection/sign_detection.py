#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2  
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
import numpy as np

class SingRec(Node):
    def __init__(self):
        super().__init__("sign_detection")
        self.sub = self.create_subscription(
            PointCloud2, 'converted_pointcloud2', self.sr_call_back, 10
        )
        self.count = 0
        self.pub = self.create_publisher(PointCloud2, 'filtered_pointcloud2', 10)

    def sr_call_back(self, msg):
        # Unpack the PointCloud2 message to get x, y, z, and intensity
        point_cloud_data = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )

        # Filter points by distance
        filtered_points = self.filter_points_by_distance(point_cloud_data, min_distance=0.5, max_distance=2)

        # Perform clustering on the filtered points
        clusters = self.perform_clustering(filtered_points)

        self.shape_filter(clusters)

        # self.get_logger().info(clusters)

        # Create and publish the filtered PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        # Define the fields
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1)
        ]

        # Create the PointCloud2 message
        filtered_pointcloud_msg = point_cloud2.create_cloud(header, fields, clusters)

        # Publish the filtered message
        self.pub.publish(filtered_pointcloud_msg)

    def filter_points_by_distance(self, points, min_distance, max_distance):
        filtered_points = []

        for point in points:
            x, y, z, intensity = point
            distance = math.sqrt(x**2 + y**2)  # Calculate the 2D Euclidean distance

            if min_distance <= distance:
                if distance <= max_distance:
                    if intensity >= 100:
                        filtered_points.append((x, y, z, intensity))

        return filtered_points

    def perform_clustering(self, points):
        # Convert points to a numpy array for clustering
        points_np = np.array([(x, y, z) for x, y, z, intensity in points])

        # Apply DBSCAN clustering
        clustering = DBSCAN(eps=0.5, min_samples=10).fit(points_np)

        # Get the cluster labels (Noise points will be labeled as -1)
        labels = clustering.labels_

        # Prepare the clustered points (Optional: filter noise points)
        clustered_points = [point for i, point in enumerate(points) if labels[i] != -1]

        return clustered_points

    def shape_filter(self, points):
        # pass
        jugement1 = False
        jugement2 = False
        self.count += 1
        self.get_logger().info(f'Frame {self.count} #####')
        for point in points:
            x, y, z, intensity = point
            for poi in points:
                poi_x, poi_y, poi_z, poi_intensity = poi
                if z + 0.4 <= poi_z <= z + 0.6:
                    jugement1 = True
                    # self.get_logger().info('j1 is True')
                if x + 0.05 <= poi_x <= x + 0.1:
                    jugement2 = True
                    # self.get_logger().info('j2 is True')

            if(jugement1 and jugement2):
                self.get_logger().info('##### Match!######')
            
            jugement1 = False
            jugement2 = False
        # arr = []
        # for point in points:
        #     (x, y, z, intensity) = point
        #     arr.append((x, y, z, intensity))
        # arr_np = np.array(arr)
        # y_ave = np.mean(arr_np,axis=0)[1]
        # self.get_logger().info(f'average : {y_ave}') 



def main(args=None):
    rclpy.init(args=args)
    node = SingRec()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
