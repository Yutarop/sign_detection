#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import open3d as o3d
from ament_index_python.packages import get_package_share_directory
import os
 
 
class ReadTemp(Node): # MODIFY NAME
    def __init__(self):
        super().__init__("read_temp") # MODIFY NAME
        package_share_directory = get_package_share_directory('sign_detection')
        pcd_path = os.path.join(package_share_directory, 'template_pcd', 'temp2.pcd')
        template_cloud = o3d.io.read_point_cloud(pcd_path)
        self.get_logger().info(f'{template_cloud}')

 
def main(args=None):
    rclpy.init(args=args)
    node = ReadTemp() # MODIFY NAME
    rclpy.spin(node)
    rclpy.shutdown()
 
if __name__ == "__main__":
    main()
