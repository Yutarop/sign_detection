#!/usr/bin/env python3
import math
import os

import numpy as np
import open3d as o3d
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN
from std_msgs.msg import Header

from sign_detection import functions

# Constants
TOPIC_IN_NAME = "pcd_segment_obs"
TOPIC_OUT_NAME = "filtered_pointcloud2"
MIN_DISTANCE = 0.5
MAX_DISTANCE = 4.0
MIN_ANGLE = 10.0
MAX_ANGLE = 170.0
MIN_INTENSITY = 130.0
DBSCAN_EPS = 0.7
DBSCAN_MIN_SAMPLES = 3
FITNESS_THRESHOLD = 0.8


class SingRec2(Node):
    def __init__(self):
        super().__init__("sign_detection")

        package_share_directory = get_package_share_directory("sign_detection")

        pcd_path = os.path.join(
            package_share_directory, "template_pcd", "template_pcd.pcd"
        )
        self.template_cloud = o3d.io.read_point_cloud(pcd_path)
        self.template_centroid = functions.compute_centroid(self.template_cloud)

        self.sub = self.create_subscription(
            PointCloud2, TOPIC_IN_NAME, self.sr_call_back, 10
        )
        self.pub = self.create_publisher(PointCloud2, TOPIC_OUT_NAME, 10)

    def sr_call_back(self, msg):
        # Unpack the PointCloud2 message to get x, y, z, and intensity
        point_cloud_data = point_cloud2.read_points(
            msg, field_names=("x", "y", "z", "intensity"), skip_nans=True
        )

        # Filter points by distance and angle
        filtered_points = self.filter_points_by_distance(point_cloud_data)

        self.perform_clustering_and_icp_matching(filtered_points)

        # Create and publish the filtered PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id

        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(
                name="intensity", offset=12, datatype=PointField.FLOAT32, count=1
            ),
        ]

        filtered_pointcloud_msg = point_cloud2.create_cloud(
            header, fields, filtered_points
        )

        self.pub.publish(filtered_pointcloud_msg)

    def filter_points_by_distance(self, points):
        filtered_points = []

        for point in points:
            x, y, z, intensity = point
            distance = math.sqrt(x**2 + y**2)
            angle = math.degrees(math.atan2(x, y))
            if MIN_DISTANCE <= distance <= MAX_DISTANCE:
                if MIN_ANGLE <= angle <= MAX_ANGLE:
                    if intensity >= MIN_INTENSITY:
                        filtered_points.append((x, y, z, intensity))

        return filtered_points

    def perform_clustering_and_icp_matching(self, points):
        # Convert point cloud data to numpy array
        points_np = np.array([(x, y, z) for x, y, z, intensity in points])
        if points_np.size == 0:
            return []

        # Perform clustering with DBSCAN
        clustering = DBSCAN(eps=DBSCAN_EPS, min_samples=DBSCAN_MIN_SAMPLES).fit(points_np)
        labels = clustering.labels_

        # Check if template_cloud is already in PointCloud format
        if isinstance(self.template_cloud, o3d.geometry.PointCloud):
            template_cloud_o3d = self.template_cloud
        else:
            template_cloud_o3d = o3d.geometry.PointCloud()
            template_cloud_o3d.points = o3d.utility.Vector3dVector(self.template_cloud)

        for cluster_id in set(labels):
            if cluster_id == -1:
                continue

            # Retrieve cluster points and convert to Open3D format
            cluster_points = points_np[labels == cluster_id]
            cluster_cloud = o3d.geometry.PointCloud()
            cluster_cloud.points = o3d.utility.Vector3dVector(cluster_points)
            cluster_centroid = functions.compute_centroid(cluster_cloud)

            # Calculate the rotation matrix
            rotation_matrix = functions.compute_rotation_matrix(
                self.template_centroid, cluster_centroid
            )

            # Rotate the template point cloud
            functions.rotate_point_cloud(self.template_cloud, rotation_matrix)

            # Calculate the translation vector
            translation = cluster_centroid - functions.compute_centroid(
                self.template_cloud
            )

            # Translate the template point cloud
            functions.translate_point_cloud(self.template_cloud, translation)

            # Apply ICP
            reg_icp = functions.apply_icp(self.template_cloud, cluster_cloud)

            # Apply the final transformation matrix from ICP
            self.template_cloud.transform(reg_icp.transformation)

            # Check the matching fitness
            fitness = reg_icp.fitness

            # If you want to display fitness score, then uncomment below
            # self.get_logger().info(f'fitness: {fitness}')

            if fitness > FITNESS_THRESHOLD:
                self.get_logger().info(f'{"="*25} Detected! {"="*25}')


def main(args=None):
    rclpy.init(args=args)
    node = SingRec2()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()