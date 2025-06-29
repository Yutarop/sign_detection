![ROS2-humble Industrial CI](https://github.com/Yutarop/sign_detection/actions/workflows/ros2_ci.yml/badge.svg)
# つくばチャレンジ2024 選択課題C    
つくばチャレンジ選択課題Cで使用される経路封鎖看板を検出するアルゴリズムのROS2パッケージです。このアルゴリズムは、3D LiDARから得られるポイントクラウドデータと反射強度のデータのみを使用します。まず、入力される`PointCloud2`メッセージを処理し、距離と角度に基づいてポイントをフィルタリングします。その後、DBSCANを使用してクラスタリングを行い、初期位置合わせとICPアルゴリズムを用いて、テンプレートポイントクラウドとクラスタをマッチングします。

## 機能
* `pcd_segment_obs`トピックから`PointCloud2`メッセージをサブスクライブします。
* ポイントを距離（0.5m〜4m）、角度（10°〜170°）、および輝度（>= 130）でフィルタリングします。
* DBSCANを使用してポイントクラウドデータをクラスタリングします。
* クラスタを事前に定義されたテンプレートポイントクラウドとICPでマッチングします。
* フィルタリングされたポイントクラウドデータを`filtered_pointcloud2`トピックにパブリッシュします。
* マッチングのフィットネススコアが0.8を超える場合に検出イベントをターミナルに記録します。

## パラメータ
#### トピック名：
* 入力：pcd_segment_obs
* 出力：filtered_pointcloud2

#### フィルタリングパラメータ：
* min_distance：0.5メートル
* max_distance：4メートル
* min_angle：10度
* max_angle：170度
* min_intensity：130  

#### DBSCANパラメータ：
* eps：0.7（クラスタリング半径）
* min_samples：3（クラスタごとの最小ポイント数）  

#### ICPマッチング：
* フィットネス閾値：0.8（検出成功の基準）

## 使用方法
#### リポジトリをクローンする
```bash
cd ~/ros2_ws/src
git clone git@github.com:Yutarop/sign_detection.git
```

#### 依存関係のインストール
```bash
cd ~/ros2_ws/src/sign_detection
pip install -r requirements.txt
```

#### ワークスペースのビルド
```bash
cd ~/ros2_ws
colcon build --packages-select sign_detection
```

#### ノードの実行
```bash
source ~/ros2_ws/install/setup.bash
ros2 run sign_detection sign_detection
```

## bagファイルで実験
#### 依存リポジトリのクローン
```bash
cd ~/ros2_ws
vcs import src < sign_detection.repos
```

#### bagファイルの実行
```bash
cd ~/ros2_ws/src/sign_detection/bag
ros2 bag play example.bag
```

#### launchファイルの実行
```bash
ros2 launch sign_detection sign_detec.launch.xml
```

#### ノードの実行
```bash
ros2 run sign_detection sign_detection
```

## デモ
<div style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/95c935a2-76e0-4582-9b0e-5b23f0aa2df9" alt="example">
</div>
