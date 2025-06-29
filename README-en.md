![ROS2-humble Industrial CI](https://github.com/Yutarop/sign_detection/actions/workflows/ros2_ci.yml/badge.svg)
# Tsukuba Challenge 2024 Task C
This is a ROS2 package for detecting path-blocking signs used in the Tsukuba Challenge 2024 Optional Task C. The algorithm processes point cloud data and intensity information from a 3D LiDAR sensor. It first filters points in PointCloud2 messages based on distance and angle, then performs clustering using DBSCAN, and matches clusters with a template point cloud using initial alignment and the ICP algorithm.

## Features
* Subscribes to PointCloud2 messages from the pcd_segment_obs topic.
* Filters points by distance (0.5m to 4m), angle (10° to 170°), and intensity (>= 130).
* Clusters point cloud data using DBSCAN.
* Matches clusters with a predefined template point cloud using ICP.
* Publishes filtered point cloud data to the filtered_pointcloud2 topic.
* Logs detection events to the terminal when the matching fitness score exceeds 0.8.

## Parameters
#### Topic Names:
* Input: pcd_segment_obs
* Output: filtered_pointcloud2

#### Filtering Parameters:

* min_distance: 0.5 meters
* max_distance: 4 meters
* min_angle: 10 degrees
* max_angle: 170 degrees
* min_intensity: 130

#### DBSCAN Parameters:

* eps: 0.7 (clustering radius)
* min_samples: 3 (minimum points per cluster)

#### ICP Matching:

* Fitness Threshold: 0.8 (criterion for successful detection)

## Usage
#### Clone the Repository
```bash
cd ~/ros2_ws/src
git clone git@github.com:Yutarop/sign_detection.git
```

#### Install Dependencies
```bash
cd ~/ros2_ws/src/sign_detection
pip install -r requirements.txt
```

#### Build the Workspace
```bash
cd ~/ros2_ws
colcon build --packages-select sign_detection
```

#### Run the Node
```bash
source ~/ros2_ws/install/setup.bash
ros2 run sign_detection sign_detection
```

## Experiment with Bag File
#### Clone Dependent Repositories
```bash
cd ~/ros2_ws
vcs import src < sign_detection.repos
```

#### Play the Bag File
```bash
cd ~/ros2_ws/src/sign_detection/bag
ros2 bag play example.bag
```

#### Run the Launch File
```bash
ros2 launch sign_detection sign_detec.launch.xml
```

#### Run the Node
```bash
ros2 run sign_detection sign_detection
```

## Demo
<div style="text-align: center;">
    <img src="https://github.com/user-attachments/assets/95c935a2-76e0-4582-9b0e-5b23f0aa2df9" alt="example">
</div>
    
