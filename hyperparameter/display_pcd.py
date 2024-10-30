import open3d as o3d
from ament_index_python.packages import get_package_share_directory
import os


package_share_directory = get_package_share_directory('sign_detection')
pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'ppp_1.pcd')
pcd = o3d.io.read_point_cloud(pcd_path)

# Visualize the point cloud
o3d.visualization.draw_geometries([pcd])