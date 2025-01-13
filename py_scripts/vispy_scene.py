import sys
import copy
import time
import signal
from threading import Thread, Event

import numpy as np

from typing import cast, TYPE_CHECKING
from scipy.spatial.transform import Rotation as R

from vispy import app
from vispy.util.quaternion import Quaternion
from vispy.scene import ViewBox
from vispy.scene.visuals import Mesh, Box, XYZAxis, create_visual_node
from vispy.visuals import LineVisual
from vispy.geometry import MeshData
from vispy.scene import transforms, SceneCanvas, TurntableCamera, FlyCamera

if __name__ == "__main__":
    from vispy_robot import Robot, RobotLink
    from vispy_robot import (
        read_mesh,
        read_stl,
        rpy_xyz_to_homogeneous,
        create_robot_links,
        matrix_to_quaternion,
    )
else:
    from .vispy_robot import Robot, RobotLink
    from .vispy_robot import (
        read_mesh,
        read_stl,
        rpy_xyz_to_homogeneous,
        create_robot_links,
        matrix_to_quaternion,
    )

if TYPE_CHECKING:
    from typing import List
    from vispy.scene import Widget
    from vispy.visuals import BoxVisual, MeshVisual

CANVAS_SIZE = (1280, 720)


class XYZAxisCustomVisual(LineVisual):
    def __init__(self, H=None, **kwargs):
        if H is None:
            pos = np.array(
                [[0, 0, 0], [10, 0, 0], [0, 0, 0], [0, 10, 0], [0, 0, 0], [0, 0, 10]]
            )
        else:
            xx, xy, xz = H[:3, 0] / np.linalg.norm(H[:3, 0]) / 10
            yx, yy, yz = H[:3, 1] / np.linalg.norm(H[:3, 1]) / 10
            zx, zy, zz = H[:3, 2] / np.linalg.norm(H[:3, 2]) / 10
            x, y, z = H[:3, 3]
            pos = np.array(
                [
                    [x, y, z],
                    [x + xx, y + xy, z + xz],
                    [x, y, z],
                    [x + yx, y + yy, z + yz],
                    [x, y, z],
                    [x + zx, y + zy, z + zz],
                ]
            )
        color = np.array(
            [
                [1, 0, 0, 1],
                [1, 0, 0, 1],
                [0, 1, 0, 1],
                [0, 1, 0, 1],
                [0, 0, 1, 1],
                [0, 0, 1, 1],
            ]
        )
        connect = "segments"
        method = "gl"

        kwargs.setdefault("pos", pos)
        kwargs.setdefault("color", color)
        kwargs.setdefault("connect", connect)
        kwargs.setdefault("method", method)

        LineVisual.__init__(self, **kwargs)

    def update_data(self, H):
        xx, xy, xz = H[:3, 0] / np.linalg.norm(H[:3, 0]) / 10
        yx, yy, yz = H[:3, 1] / np.linalg.norm(H[:3, 1]) / 10
        zx, zy, zz = H[:3, 2] / np.linalg.norm(H[:3, 2]) / 10
        x, y, z = H[:3, 3]
        pos = np.array(
            [
                [x, y, z],
                [x + xx, y + xy, z + xz],
                [x, y, z],
                [x + yx, y + yy, z + yz],
                [x, y, z],
                [x + zx, y + zy, z + zz],
            ]
        )
        LineVisual.set_data(self, pos=pos)


XYZAxisCustom = create_visual_node(XYZAxisCustomVisual)


class VispyScene:

    def __init__(self):
        self.thread_initialized = Event()

        self.thread = Thread(target=self._run, daemon=True)
        self.thread.start()

        self.thread_initialized.wait()

        matrices = [np.eye(4) for _ in range(8)]
        self.arm_l = [Robot("ArmL", create_robot_links(matrices)) for _ in range(4)]
        self.arm_r = [Robot("ArmL", create_robot_links(matrices)) for _ in range(4)]
        self.world = [XYZAxisCustom() for _ in range(4)]

        self.left_camera = [XYZAxisCustom() for _ in range(4)]
        self.right_camera = [XYZAxisCustom() for _ in range(4)]
        self.head_camera = [XYZAxisCustom() for _ in range(4)]

        for i in range(4):
            self.arm_l[i].add_to_view(self.views[i])
            self.arm_r[i].add_to_view(self.views[i])
            self.views[i].add(self.world[i])
            self.views[i].add(self.left_camera[i])
            self.views[i].add(self.right_camera[i])
            self.views[i].add(self.head_camera[i])

        self.wrist_l = XYZAxisCustom()
        self.wrist_r = XYZAxisCustom()
        self.wrists = [self.wrist_l, self.wrist_r]
        for wrist in self.wrists:
            self.view_free.add(wrist)

    def _run(self):
        app.use_app("Glfw")
        self.canvas = SceneCanvas(
            title="P4Arm 可视化", keys="interactive", size=CANVAS_SIZE, show=True
        )

        self.grid = self.canvas.central_widget.add_grid()

        # self.canvas.measure_fps()

        self.view_free = ViewBox(border_color="white", parent=self.canvas.scene)
        self.view_free.bgcolor = "#efefef"
        self.view_free.camera = TurntableCamera(up="z", fov=60)

        self.view_head = ViewBox(border_color="blue", parent=self.canvas.scene)
        self.view_head.bgcolor = "#efefef"
        self.view_head.camera = FlyCamera(fov=45, interactive=False)
        self.view_head.camera.auto_roll = False

        self.view_left_hand = ViewBox(border_color="green", parent=self.canvas.scene)
        self.view_left_hand.bgcolor = "#efefef"
        self.view_left_hand.camera = FlyCamera(fov=45, interactive=False)
        self.view_left_hand.camera.auto_roll = False

        self.view_right_hand = ViewBox(border_color="yellow", parent=self.canvas.scene)
        self.view_right_hand.bgcolor = "#efefef"
        self.view_right_hand.camera = FlyCamera(fov=45, interactive=False)
        self.view_right_hand.camera.auto_roll = False

        self.grid.padding = 6
        self.grid.add_widget(self.view_free, 0, 0)
        self.grid.add_widget(self.view_head, 0, 1)
        self.grid.add_widget(self.view_left_hand, 1, 0)
        self.grid.add_widget(self.view_right_hand, 1, 1)

        self.views = [
            self.view_free,
            self.view_head,
            self.view_left_hand,
            self.view_right_hand,
        ]

        self.canvas.show()

        self.thread_initialized.set()

        self.RxPi = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])

        try:
            app.run()
        except KeyboardInterrupt:
            print("\n捕获到 Ctrl+C，程序退出。")

    def update(self, data: dict):
        RC = np.array(
            [
                [
                    0.05418117473157347,
                    -0.8390957175899125,
                    0.5412788348226603,
                    0.19090818223902745,
                ],
                [
                    -0.9948420049853792,
                    -0.09191546145030727,
                    -0.042906095872824346,
                    0.01763956917615956,
                ],
                [
                    0.08575421518139859,
                    -0.53616221861359,
                    -0.8397477537391662,
                    0.4581743181819848,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        q = matrix_to_quaternion((RC[:3, :3] @ self.RxPi).T)

        ch = self.view_head.camera
        ch.center = RC[:3, 3]
        ch.rotation1 = q
        ch.view_changed()

        # H = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
        # RC[:3, :3] = RC[:3, :3] @ H
        for i in self.head_camera:
            i.update_data(RC)

        # RC =

        # ---- right hand ----

        cr = self.view_right_hand.camera
        Rw = np.array(data["ArmR"][7])
        RC_w = np.array(
            [
                [
                    -0.045512273899251054,
                    -0.0016406908488889593,
                    0.9989624322556168,
                    0.00213743138386762,
                ],
                [
                    -0.9989193081492075,
                    0.009510470971866969,
                    -0.045494689235136165,
                    0.019535157030371817,
                ],
                [
                    -0.009425960493651353,
                    -0.9999534284532612,
                    -0.002071760927441213,
                    0.029723014155127667,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        RC = Rw @ RC_w
        q: Quaternion = matrix_to_quaternion((RC[:3, :3] @ self.RxPi).T)

        for i in self.right_camera:
            i.update_data(RC)

        cr.center = RC[:3, 3]
        cr.rotation1 = q
        cr.view_changed()

        # ---- left hand ----

        cl = self.view_left_hand.camera
        Rw = np.array(data["ArmL"][7])
        RC_w = np.array(
            [
                [
                    0.05562608413193537,
                    0.023187541693544422,
                    0.9981823864777204,
                    0.0023704038898017627,
                ],
                [
                    -0.9982748048714419,
                    0.020108027280002272,
                    0.05516412962959982,
                    0.056661476313758834,
                ],
                [
                    -0.018792358101937412,
                    -0.9995288916029975,
                    0.02426606947677601,
                    0.02327169972408006,
                ],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

        RC = Rw @ RC_w

        q: Quaternion = matrix_to_quaternion((RC[:3, :3] @ self.RxPi).T)

        for i in self.left_camera:
            i.update_data(RC)
        cl.center = RC[:3, 3]
        cl.rotation1 = q
        cl.view_changed()

        wrist_l, wrist_r = self.wrists
        wrist_l.update_data(data["ArmL"][7])
        wrist_r.update_data(data["ArmR"][7])

        for i in range(4):
            if "ArmL" in data:
                self.arm_l[i].update(data["ArmL"])
            if "ArmR" in data:
                self.arm_r[i].update(data["ArmR"])

    def close(self):
        app.quit()


def main():
    app.use_app("Glfw")
    canvas = SceneCanvas(keys="interactive", size=(800, 600), bgcolor="#ffffff")
    view = canvas.central_widget.add_view()
    view.camera = TurntableCamera(up="z", fov=60)

    angle_90 = np.pi / 2

    links: "List[RobotLink]" = []

    matrix = np.eye(4)

    baselink = RobotLink(
        "base_link", mesh_data=read_stl("../assets/RM75B/meshes/base_link.STL")
    )
    links.append(baselink)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[0, 0, 0.2405])
    link1 = RobotLink(
        "link1",
        mesh_data=read_stl("../assets/RM75B/meshes/link1.STL"),
        matrix=matrix,
    )
    links.append(link1)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[angle_90, 0, 0], xyz=[0, 0, 0])
    link2 = RobotLink(
        "link2",
        mesh_data=read_stl("../assets/RM75B/meshes/link2.STL"),
        matrix=matrix,
    )
    links.append(link2)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[0, 0, 0.256])
    link3 = RobotLink(
        "link3",
        mesh_data=read_stl("../assets/RM75B/meshes/link3.STL"),
        matrix=matrix,
    )
    links.append(link3)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[angle_90, 0, 0], xyz=[0, 0, 0])
    link4 = RobotLink(
        "link4",
        mesh_data=read_stl("../assets/RM75B/meshes/link4.STL"),
        matrix=matrix,
    )
    links.append(link4)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[0, 0, 0.21])
    link5 = RobotLink(
        "link5",
        mesh_data=read_stl("../assets/RM75B/meshes/link5.STL"),
        matrix=matrix,
    )
    links.append(link5)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[angle_90, 0, 0], xyz=[0, 0, 0])
    link6 = RobotLink(
        "link6",
        mesh_data=read_stl("../assets/RM75B/meshes/link6.STL"),
        matrix=matrix,
    )
    links.append(link6)

    matrix = matrix @ rpy_xyz_to_homogeneous(rpy=[0, 0, 0], xyz=[0, 0, 0.144])
    link7 = RobotLink(
        "link7",
        mesh_data=read_stl("../assets/RM75B/meshes/link7.STL"),
        matrix=matrix,
    )
    links.append(link7)

    d = np.sqrt(2) / 2

    angle_90 = np.pi / 2

    matrices = [
        np.eye(4),
        rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[0, 0, 0.2405]),
        rpy_xyz_to_homogeneous(rpy=[angle_90, 0, 0], xyz=[0, 0, 0]),
        rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[0, 0, 0.256]),
        rpy_xyz_to_homogeneous(rpy=[angle_90, 0, 0], xyz=[0, 0, 0]),
        rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[0, 0, 0.21]),
        rpy_xyz_to_homogeneous(rpy=[angle_90, 0, 0], xyz=[0, 0, 0]),
        rpy_xyz_to_homogeneous(rpy=[0, 0, 0], xyz=[0, 0, 0.144]),
    ]

    for i in range(1, 8):
        print(f"{i}: ")
        print(matrices[i])

    matrices[0][:3, :3] = np.array([[0, 1, 0], [d, 0, d], [d, 0, -d]])
    matrices[0][:3, 3] = np.array([0, 0.209, 0])
    arm_l = Robot("ArmL", create_robot_links(matrices))

    matrices = [
        np.eye(4),
        rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 2 * np.pi / 3], xyz=[0, 0, 0.2405]),
        rpy_xyz_to_homogeneous(rpy=[angle_90, 0, -angle_90], xyz=[0, 0, 0]),
        rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 2 * np.pi / 3], xyz=[0, 0, 0.256]),
        rpy_xyz_to_homogeneous(rpy=[angle_90, 0, np.pi / 4], xyz=[0, 0, 0]),
        rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 2 * np.pi / 3], xyz=[0, 0, 0.21]),
        rpy_xyz_to_homogeneous(rpy=[angle_90, 0, np.pi / 3], xyz=[0, 0, 0]),
        rpy_xyz_to_homogeneous(rpy=[0, 0, 0], xyz=[0, 0, 0.144]),
    ]

    matrices[0][:3, :3] = np.array([[0, -1, 0], [-d, 0, -d], [d, 0, -d]])
    matrices[0][:3, 3] = np.array([0, -0.209, 0])
    arm_r = Robot("ArmL", create_robot_links(matrices))

    arm_l.add_to_view(view)
    arm_r.add_to_view(view)

    xyz = XYZAxis()
    view.add(xyz)

    canvas.show()
    app.run()

    # app = use_app("Glfw")
    # app.create()

    # data_source = DataSource()
    # canvas_wrapper = CanvasWrapper()
    # win = MainWindow(canvas_wrapper)

    # data_source.data_to_draw.connect(canvas_wrapper.update_data)

    # win.show()
    # app.run()

    # time.sleep(5)


# signal.signal(signal.SIGINT, signal.SIG_DFL)

if __name__ == "__main__" and sys.flags.interactive == 0:
    sc = VispyScene()

    angle_90 = np.pi / 2

    matrices = [np.eye(4) for _ in range(8)]

    data = {"ArmL": matrices.copy(), "ArmR": matrices.copy()}

    for i in range(100):
        matrices[0] = rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[i, 0, 0])
        data["ArmL"] = matrices.copy()

        matrices[0] = rpy_xyz_to_homogeneous(rpy=[-angle_90, 0, 0], xyz=[-i, 0, 0])
        data["ArmR"] = matrices.copy()

        sc.update(data=data)

        print(i)
        time.sleep(1)

    sc.close()
