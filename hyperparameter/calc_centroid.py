import open3d as o3d
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
from scipy.spatial.transform import Rotation as R

# 点群の読み込み
def load_point_cloud(file_path):
    pcd = o3d.io.read_point_cloud(file_path)
    return pcd

# 重心を計算する関数
def compute_centroid(pcd):
    points = np.asarray(pcd.points)
    centroid = np.mean(points, axis=0)
    return centroid

# 点群を回転させる関数
def rotate_point_cloud(pcd, rotation_matrix):
    points = np.asarray(pcd.points)
    rotated_points = points @ rotation_matrix.T
    pcd.points = o3d.utility.Vector3dVector(rotated_points)

# 点群を平行移動する関数
def translate_point_cloud(pcd, translation):
    points = np.asarray(pcd.points)
    points += translation
    pcd.points = o3d.utility.Vector3dVector(points)

# 回転行列を計算する関数
def compute_rotation_matrix(source_centroid, target_centroid):
    # 任意の点の間のベクトル
    vec = target_centroid - source_centroid
    angle = np.arctan2(vec[1], vec[0])  # 2Dの平面上での角度を計算

    # Z軸回りの回転行列を作成
    rotation_matrix = R.from_euler('z', angle).as_matrix()
    return rotation_matrix

# ICPを適用する関数
def apply_icp(source, target):
    threshold = 0.1  # 対応点の最大距離 0.02
    reg_icp = o3d.pipelines.registration.registration_icp(
        source, target, threshold,
        np.eye(4),  # 初期変換行列
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return reg_icp

def main(template_file, clustered_file):
    # 点群の読み込み
    template_pcd = load_point_cloud(template_file)
    clustered_pcd = load_point_cloud(clustered_file)

    # # 重心の計算
    template_centroid = compute_centroid(template_pcd)
    clustered_centroid = compute_centroid(clustered_pcd)

    # # 回転行列の計算
    rotation_matrix = compute_rotation_matrix(template_centroid, clustered_centroid)

    # # テンプレート点群を回転
    rotate_point_cloud(template_pcd, rotation_matrix)

    # 平行移動ベクトルの計算
    translation = clustered_centroid - compute_centroid(template_pcd)

    # テンプレート点群を平行移動
    translate_point_cloud(template_pcd, translation)

    # ICPを適用
    reg_icp = apply_icp(template_pcd, clustered_pcd)

    # ICPによる最終的な変換行列を適用
    template_pcd.transform(reg_icp.transformation)

    # 類似度を表示
    print("Fitness:", reg_icp.fitness)
    print("Inlier RMSE:", reg_icp.inlier_rmse)

    # 色を設定（テンプレートは青、クラスタは赤）
    template_pcd.paint_uniform_color([0, 0, 1])  # 青色
    clustered_pcd.paint_uniform_color([0, 1, 0])  # 赤色

    # 結果の表示
    o3d.visualization.draw_geometries([template_pcd, clustered_pcd],
                                       window_name='Aligned Point Clouds',
                                       width=800, height=600)

if __name__ == "__main__":
    # テンプレート点群とクラスタリングした点群のファイルパスを指定
    package_share_directory = get_package_share_directory('sign_detection')
    template_file_path = os.path.join(package_share_directory, 'template_pcd', 'ponta1.pcd')
    clustered_file_path = os.path.join(package_share_directory, 'template_pcd', 'yutaro1_bag_edit.pcd')

    main(template_file_path, clustered_file_path)