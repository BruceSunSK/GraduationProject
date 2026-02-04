# 使用open3d直接转换OBJ到PCD
import open3d as o3d

def convert_obj_to_pcd(obj_path, pcd_path, sampling_density=0.1):
    """将OBJ模型转换为点云"""
    mesh = o3d.io.read_triangle_mesh(obj_path)
    
    # 采样点云
    pcd = mesh.sample_points_uniformly(
        number_of_points=int(mesh.get_surface_area() / sampling_density)
    )
    
    # 保存
    o3d.io.write_point_cloud(pcd_path, pcd)
    print(f"转换完成: {pcd_path}")

# 使用示例
convert_obj_to_pcd('MyScene.obj', 'MyScene.pcd')