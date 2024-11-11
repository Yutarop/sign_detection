import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from ament_index_python.packages import get_package_share_directory
import os
import matplotlib.pyplot as plt

package_share_directory = get_package_share_directory('sign_detection')
pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'new2.pcd')
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

x = points[:, 0]
y = points[:, 1]

# DBSCAN
epsilon = 0.05  
min_samples = 5  
db = DBSCAN(eps=epsilon, min_samples=min_samples).fit(np.c_[x, y])
labels = db.labels_

num_clusters = len(set(labels)) - (1 if -1 in labels else 0)

preset_colors = [
    [1, 0, 0],       # 赤
    [0, 0, 1],       # 青
    [0, 1, 0],       # 緑
    [0.5, 0, 0.5],   # 紫
    [0.6, 0.3, 0],   # 茶色
    [0, 0, 0],       # 黒
    [1, 1, 0],       # 黄色
    [1, 0.5, 0]      # オレンジ
]


plt.figure(figsize=(8, 8))

for cluster_id in set(labels):
    cluster_points = (labels == cluster_id)
    color = preset_colors[cluster_id % len(preset_colors)] if cluster_id != -1 else [0.5, 0.5, 0.5]
    plt.scatter(x[cluster_points], y[cluster_points], s=5, c=[color], label=f'Cluster {cluster_id}' if cluster_id != -1 else 'Noise', alpha=0.6)

plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.title('DBSCAN Clustering of Point Cloud (X, Y only)')
plt.legend(loc='best')
plt.axis('equal')
plt.show()