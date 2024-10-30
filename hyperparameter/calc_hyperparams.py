import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score
from itertools import product
from ament_index_python.packages import get_package_share_directory
import os


package_share_directory = get_package_share_directory('sign_detection')
pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'ppp_0.pcd')
pcd = o3d.io.read_point_cloud(pcd_path)
points = np.asarray(pcd.points)

# パラメータの範囲設定
eps_values = np.linspace(0.1, 1.0, 10)  # 適宜変更
min_samples_values = range(3, 10)       # 適宜変更

# 最適なパラメータとスコアを記録する変数
best_score = -1
best_params = None
best_labels = None

# パラメータの組み合わせを評価
for eps, min_samples in product(eps_values, min_samples_values):
    dbscan = DBSCAN(eps=eps, min_samples=min_samples)
    labels = dbscan.fit_predict(points)
    
    # クラスタリングがうまく行っているか判定
    if len(set(labels)) > 1:  # ノイズ点だけでない場合
        score = silhouette_score(points, labels)
        if score > best_score:
            best_score = score
            best_params = (eps, min_samples)
            best_labels = labels

# 結果の表示
print(f"Best Params: eps={best_params[0]}, min_samples={best_params[1]}")
print(f"Best Silhouette Score: {best_score}")

# クラスターごとに同じ色を割り当てる
unique_labels = set(best_labels)
colors = plt.get_cmap("tab10")(np.linspace(0, 1, len(unique_labels)))

# 各点にクラスターごとの色を割り当てる
cluster_colors = np.array([colors[label] if label != -1 else [0, 0, 0, 1] for label in best_labels])
pcd.colors = o3d.utility.Vector3dVector(cluster_colors[:, :3])  # RGBで設定

# 点群の表示
o3d.visualization.draw_geometries([pcd])
