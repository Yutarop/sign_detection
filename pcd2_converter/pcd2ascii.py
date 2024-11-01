import open3d as o3d

# Load the PCD file
pcd = o3d.io.read_point_cloud("ppp_100.pcd")

# Save in ASCII format
o3d.io.write_point_cloud("output_ppp_100.pcd", pcd, write_ascii=True)

