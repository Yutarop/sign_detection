import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score
from itertools import product
from ament_index_python.packages import get_package_share_directory
import os

# Load the point cloud file
package_share_directory = get_package_share_directory('sign_detection')
pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'ppp_0.pcd')
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

# Set the parameter range
eps_values = np.linspace(0.1, 1.0, 10)  # Adjust as needed
min_samples_values = range(3, 10)       # Adjust as needed

# Variables to store the best parameters and score
best_score = -1
best_params = None
best_labels = None

# Evaluate parameter combinations
for eps, min_samples in product(eps_values, min_samples_values):
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(points)
    
    # Check if clustering is successful
    if len(set(labels)) > 1:  # Only if there are clusters other than noise points
        score = silhouette_score(points, labels)
        if score > best_score:
            best_score = score
            best_params = (eps, min_samples)
            best_labels = labels

# Display results
print(f"Best Params: eps={best_params[0]}, min_samples={best_params[1]}")
print(f"Best Silhouette Score: {best_score}")

# Assign colors for each cluster
unique_labels = set(best_labels)
colors = plt.get_cmap("tab10")(np.linspace(0, 1, len(unique_labels)))

# Assign colors for each point based on its cluster
cluster_colors = np.array([colors[label] if label != -1 else [0, 0, 0, 1] for label in best_labels])
pcd.colors = o3d.utility.Vector3dVector(cluster_colors[:, :3])  # Set RGB values

# Display the point cloud
o3d.visualization.draw_geometries([pcd])
