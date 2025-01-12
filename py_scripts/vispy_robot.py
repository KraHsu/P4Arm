import sys
import copy
import time

import numpy as np

from typing import cast, TYPE_CHECKING
from scipy.spatial.transform import Rotation as R

from vispy import app
from vispy.util.quaternion import Quaternion
from vispy.app import use_app, Timer, Application
from vispy.io import read_mesh
from vispy.color import Color
from vispy.scene.visuals import Mesh, Box, XYZAxis
from vispy.geometry import MeshData
from vispy.scene import transforms, SceneCanvas, TurntableCamera

from pathlib import Path

if TYPE_CHECKING:
    from typing import List
    from vispy.scene import Widget, ViewBox
    from vispy.visuals import BoxVisual, MeshVisual

CURRENT_DIR = Path(__file__).parent


def get_ab_path(fpath: str) -> str:
    return (CURRENT_DIR / fpath).resolve()


def read_stl(stdpath: str) -> "MeshData":
    vertices, faces, _, _ = read_mesh(get_ab_path(stdpath))
    return MeshData(vertices=vertices, faces=faces)


def rpy_xyz_to_homogeneous(rpy, xyz):
    rpy = np.asarray(rpy, dtype=np.float64)
    xyz = np.asarray(xyz, dtype=np.float64)

    rotation = R.from_euler("xyz", rpy)
    rotation_matrix = rotation.as_matrix()

    homogeneous_matrix = np.eye(4)
    homogeneous_matrix[:3, :3] = rotation_matrix
    homogeneous_matrix[:3, 3] = xyz

    return homogeneous_matrix


class RobotLink:
    def __init__(
        self,
        name: str = "link",
        mesh_data: "MeshData" = None,
        matrix: np.ndarray[4, 4] = np.eye(4),
    ):
        self._name = name
        self._mesh = Mesh(
            meshdata=mesh_data, color=Color(color="#3f51b5"), shading="flat"
        )
        self._mesh.transform = transforms.MatrixTransform()
        self.matrix = matrix

    @property
    def mesh(self):
        return self._mesh

    @mesh.setter
    def mesh(self, mesh: "MeshVisual"):
        self._mesh = mesh

    @property
    def transform(self) -> "transforms.MatrixTransform":
        return self._mesh.transform

    @property
    def matrix(self):
        return self._mesh.transform.matrix.T

    @matrix.setter
    def matrix(self, matrix: np.ndarray[4, 4]):
        matrix[3, :] = np.array([0, 0, 0, 1])
        self._mesh.transform.matrix = matrix.T

    def add_to_view(self, view: "ViewBox"):
        view.add(self._mesh)


class Robot:
    def __init__(self, name: str, links: "List[RobotLink]"):
        self._name = name
        self._links = links

    def update(self, matrices: "List[np.ndarray[4,4]]"):
        for i in range(8):
            self._links[i].matrix = matrices[i]

    @property
    def links(self):
        return self._links

    def __getitem__(self, key):
        return self._links[key]

    def add_to_view(self, view: "ViewBox"):
        for link in self._links:
            link.add_to_view(view)


def create_robot_links(matrices: "List[np.ndarray[4,4]]"):
    links = []

    # 定义每个链接对应的 STL 文件路径
    mesh_files = [
        "../assets/RM75B/meshes/base_link.STL",
        "../assets/RM75B/meshes/link1.STL",
        "../assets/RM75B/meshes/link2.STL",
        "../assets/RM75B/meshes/link3.STL",
        "../assets/RM75B/meshes/link4.STL",
        "../assets/RM75B/meshes/link5.STL",
        "../assets/RM75B/meshes/link6.STL",
        "../assets/RM75B/meshes/link7.STL",
    ]

    # 创建每个 RobotLink 对象并加入列表
    matrix_ = np.eye(4)

    for i, matrix in enumerate(matrices):
        matrix_ = matrix_ @ matrix
        link = RobotLink(f"link{i}", mesh_data=read_stl(mesh_files[i]), matrix=matrix_)
        links.append(link)

    return links


def matrix_to_quaternion(matrix):
    # 检查输入是否为 3x3 矩阵
    if matrix.shape != (3, 3):
        raise ValueError("输入的旋转矩阵必须是 3x3 矩阵")

    # 计算旋转矩阵的迹
    trace = np.trace(matrix)

    if trace > 0:
        # 如果迹大于 0
        s = 2.0 * np.sqrt(1.0 + trace)
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    else:
        # 根据最大对角线元素选择计算路径
        if matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2])
            w = (matrix[2, 1] - matrix[1, 2]) / s
            x = 0.25 * s
            y = (matrix[0, 1] + matrix[1, 0]) / s
            z = (matrix[0, 2] + matrix[2, 0]) / s
        elif matrix[1, 1] > matrix[2, 2]:
            s = 2.0 * np.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2])
            w = (matrix[0, 2] - matrix[2, 0]) / s
            x = (matrix[0, 1] + matrix[1, 0]) / s
            y = 0.25 * s
            z = (matrix[1, 2] + matrix[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1])
            w = (matrix[1, 0] - matrix[0, 1]) / s
            x = (matrix[0, 2] + matrix[2, 0]) / s
            y = (matrix[1, 2] + matrix[2, 1]) / s
            z = 0.25 * s

    # 返回四元数
    return Quaternion(w, x, y, z)
