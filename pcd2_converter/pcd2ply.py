import open3d as o3d

pcd = o3d.io.read_point_cloud("ppp_100.pcd")

o3d.io.write_point_cloud("output.ply", pcd)
