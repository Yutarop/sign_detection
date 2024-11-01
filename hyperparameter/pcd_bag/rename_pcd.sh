#!/bin/bash

DIRECTORY=~/ros2_ws/src/sign_detection/hyperparameter/pcd_bag

count=218
for file in $(ls "$DIRECTORY"/ooo_*.pcd | sort); do
    new_name=$(printf "ppp_%d.pcd" "$count")
    mv "$file" "$DIRECTORY/$new_name"
    count=$((count + 1))
done

echo "Done with changing names"
