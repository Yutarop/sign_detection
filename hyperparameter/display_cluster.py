import open3d as o3d
import numpy as np
from sklearn.cluster import DBSCAN
from ament_index_python.packages import get_package_share_directory
import os

package_share_directory = get_package_share_directory('sign_detection')
pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'new1.pcd')
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

eps = 0.6
min_samples = 4
dbscan = DBSCAN(eps=eps, min_samples=min_samples)
labels = dbscan.fit_predict(points)

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

cluster_colors = np.array([preset_colors[label % len(preset_colors)] if label != -1 else [0, 0, 0] for label in labels])
pcd.colors = o3d.utility.Vector3dVector(cluster_colors)
o3d.visualization.draw_geometries([pcd])
