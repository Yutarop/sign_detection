#!/usr/bin/env python3
import math
import rclpy
import numpy as np
import open3d as o3d
import os
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2  
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from ament_index_python.packages import get_package_share_directory


class SingRec2(Node):
    def __init__(self):
        super().__init__("sign_detection2")

        package_share_directory = get_package_share_directory('sign_detection')
        pcd_path = os.path.join(package_share_directory, 'template_pcd', 'temp3.pcd')
        self.template_cloud = o3d.io.read_point_cloud(pcd_path)
        
        self.sub = self.create_subscription(
            PointCloud2, 'pcd_segment_obs', self.sr_call_back, 10
        )
        self.pub = self.create_publisher(PointCloud2, 'filtered_pointcloud2', 10)

    def sr_call_back(self, msg):
        # Unpack the PointCloud2 message to get x, y, z, and intensity
        point_cloud_data = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )

        # Filter points by distance
        filtered_points = self.filter_points_by_distance(point_cloud_data, min_distance=0.5, max_distance=4)

        self.perform_clustering_and_icp_matching(filtered_points)

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
        filtered_pointcloud_msg = point_cloud2.create_cloud(header, fields, filtered_points)

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

    def perform_clustering_and_icp_matching(self, points, eps=0.6, min_samples=4, threshold=0.02):
        # Convert point cloud data to numpy array
        points_np = np.array([(x, y, z) for x, y, z, intensity in points])
        if points_np.size == 0:
            return []
        
        # Perform clustering with DBSCAN
        clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(points_np)
        labels = clustering.labels_

        # No need to re-assign points if template_cloud is already in PointCloud format
        if isinstance(self.template_cloud, o3d.geometry.PointCloud):
            template_cloud_o3d = self.template_cloud  # Use directly if already a PointCloud
        else:
            template_cloud_o3d = o3d.geometry.PointCloud()
            template_cloud_o3d.points = o3d.utility.Vector3dVector(self.template_cloud)

        matched_clusters = []
        # self.get_logger().info(f'{len(set(labels))}')
        for cluster_id in set(labels):
            if cluster_id == -1:
                continue  # Skip noise points

            # Retrieve cluster points and convert to Open3D format
            cluster_points = points_np[labels == cluster_id]
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)

            # Apply ICP
            icp_result = o3d.pipelines.registration.registration_icp(
                cluster_cloud, template_cloud_o3d, threshold,
                estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint()
            )

            # Check the matching fitness
            fitness = icp_result.fitness
            if fitness > 0:  # Example threshold for a match
                # self.get_logger().info('##### Match!######')
                self.get_logger().info(f'fitness: {fitness}')
            # self.get_logger().info(f'fitness: {fitness}')


def main(args=None):
    rclpy.init(args=args)
    node = SingRec2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
