import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
from sklearn.metrics import silhouette_score
from ament_index_python.packages import get_package_share_directory
import os

package_share_directory = get_package_share_directory('sign_detection')

# Epsilonを0.7に固定
eps_fixed = 0.7

# min_samplesの範囲設定
min_samples_values = range(3, 9)  # min_samples = 3, 4, 5, 6, 7, 8

# 各min_samplesに対する全データのシルエットスコアの合計とカウントを保存する辞書
silhouette_scores_sum = {min_samples: 0 for min_samples in min_samples_values}
silhouette_scores_count = {min_samples: 0 for min_samples in min_samples_values}

# 各データファイル（375個）に対してクラスタリングとシルエットスコア計算
for i in range(199):
    pcd_path = os.path.join(package_share_directory, 'hyperparameter', 'ppp_' + str(i) + '.pcd')
    pcd = o3d.io.read_point_cloud(pcd_path)
    points = np.asarray(pcd.points)

    # min_samplesごとにDBSCANクラスタリングを実行
    for min_samples in min_samples_values:
        dbscan = DBSCAN(eps=eps_fixed, min_samples=min_samples)
        labels = dbscan.fit_predict(points)
        
        # クラスタリングがうまく行っているか判定
        if len(set(labels)) > 1:  # ノイズ点だけでない場合
            score = silhouette_score(points, labels)
            silhouette_scores_sum[min_samples] += score
            silhouette_scores_count[min_samples] += 1

# 各min_samplesごとに平均シルエットスコアを計算
average_silhouette_scores = {
    min_samples: (silhouette_scores_sum[min_samples] / silhouette_scores_count[min_samples])
    for min_samples in min_samples_values
}

# グラフのプロット
plt.figure(figsize=(8, 5))
plt.plot(list(average_silhouette_scores.keys()), list(average_silhouette_scores.values()), marker='o')
plt.xlabel('Min Samples')
plt.ylabel('Average Silhouette Score')
plt.title(f'Average Silhouette Score vs. Min Samples (Epsilon = {eps_fixed})')
plt.grid()
plt.show()
