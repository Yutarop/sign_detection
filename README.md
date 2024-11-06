# Tsukuba Challenge - つくばチャレンジ選択課題C -
### Introduction
This is a package for detecting road closed signs for Task C in the Tsukuba Challenge. It detects signs using reflection intensity information from 3D LiDAR.

## Setup
```bash
$ cd ~/{ROS2_WORKSPACE}/src
$ git clone -b release/humble https://github.com/Yutarop/sign_detection.git
$ cd ~/{ROS2_WORKSPACE} && $ colcon build
```

## Run
```bash
$ cd ~/{ROS2_WORKSPACE}/src/sign_detection/sign_detection
$ python3 main.py
```
