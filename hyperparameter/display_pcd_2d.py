import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from ament_index_python.packages import get_package_share_directory
import os
import matplotlib.pyplot as plt

package_share_directory = get_package_share_directory('sign_detection')
pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'new1.pcd')
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

x = points[:, 0]
y = points[:, 1]

plt.figure(figsize=(8, 8))
plt.scatter(x, y, s=1, c='blue', alpha=0.5)
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.title('Point Cloud (X, Y only)')
plt.axis('equal')
plt.show()