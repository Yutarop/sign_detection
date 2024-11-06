import open3d as o3d
import numpy as np
from scipy.spatial.transform import Rotation as R

# Function to compute the centroid
def compute_centroid(pcd):
    points = np.asarray(pcd.points)
    centroid = np.mean(points, axis=0)
    return centroid

# Function to rotate a point cloud
def rotate_point_cloud(pcd, rotation_matrix):
    points = np.asarray(pcd.points)
    rotated_points = points @ rotation_matrix.T  # Using the transpose of the matrix
    pcd.points = o3d.utility.Vector3dVector(rotated_points)

# Function to translate a point cloud
def translate_point_cloud(pcd, translation):
    points = np.asarray(pcd.points)
    points += translation
    pcd.points = o3d.utility.Vector3dVector(points)

# Function to compute the rotation matrix
def compute_rotation_matrix(source_centroid, target_centroid):
    # Vector between two arbitrary points
    vec = target_centroid - source_centroid
    angle = np.arctan2(vec[1], vec[0])  # Calculate angle in 2D plane

    # Create rotation matrix around the Z-axis
    rotation_matrix = R.from_euler('z', angle).as_matrix()
    return rotation_matrix

# Function to apply ICP
def apply_icp(source, target):
    threshold = 0.06  # Maximum distance for corresponding points
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold,
        np.eye(4),  # Initial transformation matrix
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return reg_icp
