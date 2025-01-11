# import trimesh
# import gzip

# from os import path as os_path

# from vispy.io import read_mesh


# def read_mesh_from_stl(stl_path: str):

#     fmt = os_path.splitext(stl_path)[1].lower()
#     print(fmt)
#     if fmt != ".stl":
#         fmt = os_path.splitext(os_path.splitext(stl_path)[0])[1].lower()

#     # return read_mesh()


# def stl_to_obj_gz(stl_file, obj_gz_file):
#     mesh = trimesh.load(stl_file)

#     if mesh.is_empty:
#         raise ValueError(f"无法加载 STL 文件: {stl_file}")

#     obj_data = trimesh.exchange.obj.export_obj(mesh)

#     with gzip.open(obj_gz_file, "wt") as f:
#         f.write(obj_data)

#     print(f"成功将 {stl_file} 转换为 {obj_gz_file}")


# if __name__ == "__main__":
#     read_mesh_from_stl(
#         R"/home/charleshsu/workspace/P4/Arm/assets/RM75/urdf/rm_75_b_description/meshes/link1.STL"
#     )
