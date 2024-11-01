import open3d as o3d
import numpy as np
import os
from ament_index_python.packages import get_package_share_directory

def preprocess_point_cloud_without_downsampling(pcd, radius_normal, radius_feature):
    # 法線推定
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    # FPFH特徴量を計算
    fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        pcd,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100)
    )
    return pcd, fpfh


def execute_global_registration(source, target, source_fpfh, target_fpfh, distance_threshold):
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source, target, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False), 4,
        [o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(0.9),
         o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(distance_threshold)],
        o3d.pipelines.registration.RANSACConvergenceCriteria(4000000, 500)
    )
    return result


def refine_registration(source, target, transformation, distance_threshold):
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, transformation,
        o3d.pipelines.registration.TransformationEstimationPointToPoint()
    )
    return result


def main():
    # パッケージディレクトリとテンプレートの読み込み
    package_share_directory = get_package_share_directory('sign_detection')
    template_pcd_path = os.path.join(package_share_directory, 'template_pcd', 'yutaro3_bag_edit.pcd')
    target_pcd_path = os.path.join(package_share_directory, 'template_pcd', 'yutaro2_bag_edit.pcd')  # 認識したいシーンのPCDファイル

    template_cloud = o3d.io.read_point_cloud(template_pcd_path)
    target_cloud = o3d.io.read_point_cloud(target_pcd_path)

    # パラメータ設定
    radius_normal = 0.1  # 法線計算の半径
    radius_feature = 0.25  # FPFH特徴量計算の半径
    distance_threshold = 0.05  # 初期位置合わせの距離閾値 0.3

    # 特徴点の抽出とFPFH特徴量の計算（ダウンサンプリングなし）
    template_cloud, template_fpfh = preprocess_point_cloud_without_downsampling(template_cloud, radius_normal, radius_feature)
    target_cloud, target_fpfh = preprocess_point_cloud_without_downsampling(target_cloud, radius_normal, radius_feature)

    # Global Registrationによる初期位置合わせ
    initial_transformation = execute_global_registration(template_cloud, target_cloud, template_fpfh, target_fpfh, distance_threshold)
    print("Initial Transformation:", initial_transformation.transformation)

    # ICPでの微調整
    refined_result = refine_registration(template_cloud, target_cloud, initial_transformation.transformation, distance_threshold)
    print("Refined Transformation:", refined_result.transformation)

    # ICPのフィットネスとRMSEを表示
    fitness = refined_result.fitness
    inlier_rmse = refined_result.inlier_rmse
    print(f"ICP Fitness: {fitness:.4f}")
    print(f"ICP Inlier RMSE: {inlier_rmse:.4f}")

    # 合成したテンプレートとターゲットの点群を表示
    template_cloud.transform(refined_result.transformation)  # テンプレートに最終変換を適用
    template_cloud.paint_uniform_color([0, 0, 1])  # 青色
    target_cloud.paint_uniform_color([1, 0, 0])  # 赤色
    o3d.visualization.draw_geometries([template_cloud, target_cloud])


if __name__ == "__main__":
    main()
