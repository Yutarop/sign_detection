#!/bin/bash

DIRECTORY=~/ros2_ws/src/sign_detection/hyperparameter/pcd_bag

count=$(ls "$DIRECTORY"/*.pcd 2>/dev/null | wc -l)

echo "The number of pcd files in the directory: $count"
