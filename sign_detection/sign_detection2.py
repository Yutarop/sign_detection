#!/usr/bin/env python3
import math
import rclpy
import numpy as np
import open3d as o3d
import os
from sign_detection import functions
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2  
from std_msgs.msg import Header
from sklearn.cluster import DBSCAN
from ament_index_python.packages import get_package_share_directory


class SingRec2(Node):
    def __init__(self, detection_queue):
        super().__init__("sign_detection2")
        self.detection_queue = detection_queue

        package_share_directory = get_package_share_directory('sign_detection')

        pcd_path = os.path.join(package_share_directory, 'template_pcd', 'bag1_edit.pcd')
        self.template_cloud = o3d.io.read_point_cloud(pcd_path)
        self.template_centroid = functions.compute_centroid(self.template_cloud)

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
        filtered_points = self.filter_points_by_distance(point_cloud_data, min_distance=0.5, max_distance=4, min_angle=10, max_angle=170)

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

    def filter_points_by_distance(self, points, min_distance, max_distance, min_angle, max_angle):
        filtered_points = []

        for point in points:
            x, y, z, intensity = point
            distance = math.sqrt(x**2 + y**2)
            angle = math.degrees(math.atan2(x, y))
            if min_distance <= distance <= max_distance:
                if min_angle <= angle <= max_angle:
                    # if intensity >= 130:
                    filtered_points.append((x, y, z, intensity))

        return filtered_points

    def perform_clustering_and_icp_matching(self, points, eps=0.7, min_samples=3):
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

        for cluster_id in set(labels):
            if cluster_id == -1:
                continue  # Skip noise points

            # Retrieve cluster points and convert to Open3D format
            cluster_points = points_np[labels == cluster_id]
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)
            cluster_centroid = functions.compute_centroid(cluster_cloud)

            # Calculate the rotation matrix
            rotation_matrix = functions.compute_rotation_matrix(self.template_centroid, cluster_centroid)

            # Rotate the template point cloud
            functions.rotate_point_cloud(self.template_cloud , rotation_matrix)

            # Calculate the translation vector
            translation = cluster_centroid - functions.compute_centroid(self.template_cloud)

            # Translate the template point cloud
            functions.translate_point_cloud(self.template_cloud, translation)

            # Apply ICP
            reg_icp = functions.apply_icp(self.template_cloud, cluster_cloud)

            # Apply the final transformation matrix from ICP
            self.template_cloud.transform(reg_icp.transformation)

            # Check the matching fitness
            fitness = reg_icp.fitness
            if fitness > 0.9:
                self.get_logger().info(f'fitness: {fitness}')
                self.detection_queue.put("Detected")
            else:
                self.detection_queue.put("Waiting...")


def main(args=None, detection_queue=None):
    rclpy.init(args=args)
    node = SingRec2(detection_queue)
    rclpy.spin(node)
    rclpy.shutdown()
