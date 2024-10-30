import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score
from itertools import product
from ament_index_python.packages import get_package_share_directory
import os


package_share_directory = get_package_share_directory('sign_detection')

# 最適なパラメータを格納するリスト
optimal_eps_values = []
optimal_min_samples_values = []

# パラメータの範囲設定
eps_values = np.linspace(0.1, 1.0, 10)  # 適宜変更
min_samples_values = range(3, 10)       # 適宜変更

for i in range(217):
    pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'ppp_' + str(i) + '.pcd')
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

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

    # 最適なパラメータをリストに追加
    if best_params:
        optimal_eps_values.append(best_params[0])
        optimal_min_samples_values.append(best_params[1])

# 各パラメータの平均を計算
average_eps = np.mean(optimal_eps_values)
average_min_samples = np.mean(optimal_min_samples_values)

# 結果の表示
print(f"Optimal Parameters for Each File:")
for i in range(217):
    print(f"File: ppp_{i}.pcd, Best eps: {optimal_eps_values[i]}, Best min_samples: {optimal_min_samples_values[i]}")

print("\nAverage Optimal Parameters Across All Files:")
print(f"Average eps: {average_eps}")
print(f"Average min_samples: {average_min_samples}")

# Average Optimal Parameters Across All Files:
# Average eps: 0.6018433179723504
# Average min_samples: 3.6267281105990783

