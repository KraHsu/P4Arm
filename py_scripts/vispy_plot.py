import numpy as np
from vispy import app, scene
from vispy.visuals.transforms import MatrixTransform
import threading

app.use_app("Glfw")


class Point:
    def __init__(self, name, P):
        self.name = name
        self.P = P.astype(np.float32)

    def get_origin(self):
        return self.P

    def get_visualization_data(self, scale=1.0):
        """返回点的可视化数据

        Returns:
            dict: 包含可视化所需的所有数据
                - lines: 线段数据 (N, 3) 或 None
                - line_colors: 线段颜色 (N, 4) 或 None
                - origin: 原点位置 (3,)
                - origin_color: 原点颜色 (4,)
                - label: 标签文本
                - label_pos: 标签位置 (3,)
        """
        return {
            "lines": None,
            "line_colors": None,
            "origin": self.P,
            "origin_color": np.array([0, 0, 1, 1]),  # 蓝色
            "label": self.name,
            "label_pos": self.P,
        }


class Line:
    def __init__(self, name, start, end):
        """创建一个线段

        Args:
            name (str): 线段名称
            start (np.ndarray): 起点坐标 (3,)
            end (np.ndarray): 终点坐标 (3,)
        """
        self.name = name
        self.start = start.astype(np.float32)
        self.end = end.astype(np.float32)

    def get_origin(self):
        return self.start

    def get_visualization_data(self, scale=1.0):
        """返回线段的可视化数据"""
        midpoint = (self.start + self.end) / 2  # 用于放置标签

        return {
            "lines": np.array([self.start, self.end], dtype=np.float32),
            "line_colors": np.array(
                [[0.7, 0.7, 0, 1], [0.7, 0.7, 0, 1]], dtype=np.float32
            ),  # 黄色
            "origin": midpoint,
            "origin_color": np.array([0.7, 0.7, 0, 1]),  # 黄色
            "label": self.name,
            "label_pos": midpoint,
        }


class Plane:
    def __init__(self, name, origin, normal, size=0.2):
        """创建一个平面

        Args:
            name (str): 平面名称
            origin (np.ndarray): 平面中心点 (3,)
            normal (np.ndarray): 平面法向量 (3,)
            size (float): 平面可视化大小
        """
        self.name = name
        self.origin = origin.astype(np.float32)
        self.normal = normal.astype(np.float32)
        self.normal = self.normal / np.linalg.norm(self.normal)  # 归一化法向量
        self.size = size

        # 计算平面上的两个正交基向量
        self.u = (
            np.array([1, 0, 0])
            if not np.allclose(self.normal, [1, 0, 0])
            else np.array([0, 1, 0])
        )
        self.u = self.u - np.dot(self.u, self.normal) * self.normal
        self.u = self.u / np.linalg.norm(self.u)
        self.v = np.cross(self.normal, self.u)

    def get_origin(self):
        return self.origin

    def get_visualization_data(self, scale=1.0):
        """返回平面的可视化数据"""
        # 创建平面的四个角点
        corners = [
            self.origin + self.size * (-self.u - self.v),
            self.origin + self.size * (self.u - self.v),
            self.origin + self.size * (self.u + self.v),
            self.origin + self.size * (-self.u + self.v),
        ]

        # 创建平面边界线
        lines = []
        for i in range(4):
            lines.extend([corners[i], corners[(i + 1) % 4]])

        # 添加法向量
        normal_endpoint = self.origin + self.size * self.normal
        lines.extend([self.origin, normal_endpoint])

        # 创建颜色数组（紫色边界 + 红色法向量）
        colors = [[0.8, 0, 0.8, 1]] * 8  # 紫色边界
        colors.extend([[1, 0, 0, 1]] * 2)  # 红色法向量

        return {
            "lines": np.array(lines, dtype=np.float32),
            "line_colors": np.array(colors, dtype=np.float32),
            "origin": self.origin,
            "origin_color": np.array([0.8, 0, 0.8, 1]),  # 紫色
            "label": self.name,
            "label_pos": self.origin,
        }


class Rectangle:
    def __init__(self, name, center, normal, width, height):
        """创建一个矩形

        Args:
            name (str): 矩形名称
            center (np.ndarray): 矩形中心点 (3,)
            normal (np.ndarray): 矩形法向量 (3,)
            width (float): 矩形宽度
            height (float): 矩形高度
        """
        self.name = name
        self.center = center.astype(np.float32)
        self.normal = normal.astype(np.float32)
        self.normal = self.normal / np.linalg.norm(self.normal)
        self.width = width
        self.height = height

        # 计算矩形平面的两个方向向量
        self.u = (
            np.array([1, 0, 0])
            if not np.allclose(self.normal, [1, 0, 0])
            else np.array([0, 1, 0])
        )
        self.u = self.u - np.dot(self.u, self.normal) * self.normal
        self.u = self.u / np.linalg.norm(self.u)
        self.v = np.cross(self.normal, self.u)

    def get_origin(self):
        return self.center

    def get_visualization_data(self, scale=1.0):
        """返回矩形的可视化数据"""
        # 计算四个角点
        half_width = self.width / 2
        half_height = self.height / 2
        corners = [
            self.center + (-half_width * self.u - half_height * self.v),
            self.center + (half_width * self.u - half_height * self.v),
            self.center + (half_width * self.u + half_height * self.v),
            self.center + (-half_width * self.u + half_height * self.v),
        ]

        # 创建边界线
        lines = []
        for i in range(4):
            lines.extend([corners[i], corners[(i + 1) % 4]])

        # 添加法向量
        normal_endpoint = self.center + 0.1 * self.normal  # 较短的法向量
        lines.extend([self.center, normal_endpoint])

        # 创建颜色数组（绿色边界 + 红色法向量）
        colors = [[0, 0.7, 0, 1]] * 8  # 绿色边界
        colors.extend([[1, 0, 0, 1]] * 2)  # 红色法向量

        return {
            "lines": np.array(lines, dtype=np.float32),
            "line_colors": np.array(colors, dtype=np.float32),
            "origin": self.center,
            "origin_color": np.array([0, 0.7, 0, 1]),  # 绿色
            "label": self.name,
            "label_pos": self.center,
        }


class Frame:
    def __init__(self, name, H):
        self.name = name
        self.H = H.astype(np.float32)

    def get_origin(self):
        return self.H[:3, 3].flatten()

    def get_visualization_data(self, scale=1.0):
        """返回坐标系的可视化数据

        Returns:
            dict: 包含可视化所需的所有数据
                - lines: 线段数据 (N, 3)
                - line_colors: 线段颜色 (N, 4)
                - origin: 原点位置 (3,)
                - origin_color: 原点颜色 (4,)
                - label: 标签文本
                - label_pos: 标签位置 (3,)
        """
        origin = self.H[:3, 3]
        R = self.H[:3, :3]

        # 构建坐标轴线段
        axes_lines = []
        line_colors = []

        # 为每个轴创建线段和对应颜色
        colors = [
            [1, 0, 0, 1],  # X轴红色
            [0, 1, 0, 1],  # Y轴绿色
            [0, 0, 1, 1],  # Z轴蓝色
        ]

        for i in range(3):
            endpoint = origin + scale * R[:, i]
            axes_lines.extend([origin, endpoint])
            line_colors.extend([colors[i]] * 2)

        return {
            "lines": np.array(axes_lines, dtype=np.float32),
            "line_colors": np.array(line_colors, dtype=np.float32),
            "origin": origin,
            "origin_color": np.array([0, 0, 0, 1]),  # 黑色
            "label": self.name,
            "label_pos": origin,
        }


class Realtime3DScene:
    def __init__(self):
        self._running = True
        self.app_thread = threading.Thread(target=self._run_app)
        self.ready = threading.Event()
        self.app_thread.start()
        self.ready.wait()

    def _create_grid_labels(self):
        """创建网格刻度标签"""
        grid_range = np.arange(-0.5, 0.6, 0.1)  # -0.5到0.5，步长0.1

        # 创建标签容器
        self.x_labels = scene.Text(parent=self.view.scene, color="black", font_size=8)
        self.y_labels = scene.Text(parent=self.view.scene, color="black", font_size=8)
        self.z_labels = scene.Text(parent=self.view.scene, color="black", font_size=8)

        # 生成标签文本和位置
        x_texts, x_pos = [], []
        y_texts, y_pos = [], []
        z_texts, z_pos = [], []

        for i in grid_range:
            val = round(i, 1)  # 四舍五入到一位小数
            if val != 0:  # 跳过0以避免与轴重叠
                # X轴标签
                x_texts.append(str(val))
                x_pos.append((val, -0.02, -0.02))  # 稍微偏移以避免与网格重叠

                # Y轴标签
                y_texts.append(str(val))
                y_pos.append((-0.02, val, -0.02))

                # Z轴标签
                z_texts.append(str(val))
                z_pos.append((-0.02, -0.02, val))

        # 设置标签
        self.x_labels.text = x_texts
        self.x_labels.pos = x_pos
        self.y_labels.text = y_texts
        self.y_labels.pos = y_pos
        self.z_labels.text = z_texts
        self.z_labels.pos = z_pos

    def _run_app(self):
        """在单独的线程中运行 VisPy 应用"""
        # 创建主画布
        self.canvas = scene.SceneCanvas(
            keys="interactive", size=(800, 600), bgcolor="#ffffff", app="Glfw"
        )

        # 创建3D视图
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.TurntableCamera(up="z", fov=60)

        # 添加坐标轴和标签
        axis = scene.XYZAxis(parent=self.view.scene)

        # 添加带颜色的网格线
        grid = scene.GridLines(
            parent=self.view.scene, scale=(0.1, 0.1), color=(0.5, 0.5, 0.5, 0.5)
        )  # 灰色半透明

        # 添加垂直于x轴的网格
        grid_yz = scene.GridLines(
            parent=self.view.scene, scale=(0.1, 0.1), color=(0.5, 0.5, 0.5, 0.3)
        )
        grid_yz.transform = scene.transforms.MatrixTransform()
        grid_yz.transform.rotate(90, (0, 1, 0))

        # 添加垂直于y轴的网格
        grid_xz = scene.GridLines(
            parent=self.view.scene, scale=(0.1, 0.1), color=(0.5, 0.5, 0.5, 0.3)
        )
        grid_xz.transform = scene.transforms.MatrixTransform()
        grid_xz.transform.rotate(90, (1, 0, 0))

        # 添加网格刻度标签
        self._create_grid_labels()

        # 创建视觉对象
        self.lines = scene.visuals.Line(
            parent=self.view.scene, width=2, connect="segments", antialias=True
        )

        self.origins = scene.visuals.Markers(parent=self.view.scene)
        self.labels = scene.visuals.Text(
            parent=self.view.scene,
            color="black",
            face="OpenSans",
            anchor_x="left",
            anchor_y="bottom",
        )

        # 设置初始视图范围
        self.view.camera.set_range(x=[-0.5, 0.5], y=[-0.5, 0.5], z=[-0.5, 0.5])

        # 设置网格的GL状态
        grid.set_gl_state("translucent", depth_test=False)
        grid_yz.set_gl_state("translucent", depth_test=False)
        grid_xz.set_gl_state("translucent", depth_test=False)

        self.scale = 0.05
        self.canvas.show()

        self.ready.set()

        # 添加检查停止标志的定时器
        self._check_stop_timer = app.Timer(interval=0.1, connect=self._check_stop)
        self._check_stop_timer.start()

        app.run()

    def _check_stop(self, event):
        """检查是否需要停止应用"""
        if not self._running:
            self.canvas.close()
            app.quit()

    def close(self):
        """停止3D场景的方法"""
        self._running = False
        if self.app_thread.is_alive():
            self.app_thread.join()

    def update(self, objs):
        """更新场景中的所有对象

        Args:
            objs: 对象列表，每个对象都必须实现get_visualization_data方法
        """
        if not objs or not self._running:
            return

        all_lines = []
        all_line_colors = []
        origins = []
        origin_colors = []
        labels = []
        label_positions = []

        # 收集所有对象的可视化数据
        for obj in objs:
            vis_data = obj.get_visualization_data(self.scale)

            # 处理线段数据
            if vis_data["lines"] is not None:
                all_lines.extend(vis_data["lines"])
                all_line_colors.extend(vis_data["line_colors"])

            # 处理原点和标签数据
            origins.append(vis_data["origin"])
            origin_colors.append(vis_data["origin_color"])
            labels.append(vis_data["label"])
            label_positions.append(vis_data["label_pos"])

        def _update():
            if not self._running:
                return

            # 更新线段
            if all_lines:
                self.lines.set_data(
                    pos=np.array(all_lines, dtype=np.float32),
                    color=np.array(all_line_colors, dtype=np.float32),
                )
            else:
                self.lines.set_data(
                    pos=np.zeros((0, 3), dtype=np.float32),
                    color=np.zeros((0, 4), dtype=np.float32),
                )

            # 更新原点标记
            self.origins.set_data(
                pos=np.array(origins, dtype=np.float32),
                edge_color=None,
                face_color=np.array(origin_colors, dtype=np.float32),
                size=10,
            )

            # 更新标签
            self.labels.text = labels
            self.labels.pos = np.array(label_positions, dtype=np.float32)
            self.canvas.update()

        app.Timer(connect=lambda _: _update(), iterations=1).start()


def random_transform_matrix():
    from scipy.spatial.transform import Rotation

    R = Rotation.random().as_matrix()
    t = np.random.uniform(-0.3, 0.3, 3)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t
    return H


def main():
    import numpy as np
    import time

    # 创建3D场景
    scene_3d = Realtime3DScene()

    try:
        # 创建固定的参考坐标系
        reference_frames = [
            Frame("World", np.eye(4)),  # 世界坐标系
            Frame("Reference_1", random_transform_matrix()),  # 固定参考系1
        ]

        # 创建一些固定的几何对象
        static_geometries = [
            # 创建XY平面
            Plane("XY_Plane", np.array([0, 0, 0]), np.array([0, 0, 1])),
            # 创建一个固定的矩形
            Rectangle(
                "Static_Rect", np.array([0.2, 0.2, 0.1]), np.array([0, 0, 1]), 0.1, 0.15
            ),
            # 创建一条固定的线段
            Line("Static_Line", np.array([-0.2, -0.2, 0]), np.array([-0.1, -0.1, 0.2])),
        ]

        # # 主循环
        # for i in range(30 * 10):  # 运行10秒，30fps
        objects = []

        # 添加固定的参考坐标系和几何体
        objects.extend(reference_frames)
        objects.extend(static_geometries)

        #     # 添加一个运动的坐标系
        #     H = random_transform_matrix()
        #     objects.append(Frame(f"Moving_Frame", H))

        #     # 添加一个运动的点
        #     P = np.random.uniform(-0.3, 0.3, 3)
        #     objects.append(Point("Moving_Point", P))

        #     # 添加一个运动的线段
        #     start = np.random.uniform(-0.3, 0.3, 3)
        #     end = start + np.random.uniform(-0.1, 0.1, 3)
        #     objects.append(Line("Moving_Line", start, end))

        #     # 添加一个运动的平面
        #     origin = np.random.uniform(-0.3, 0.3, 3)
        #     normal = np.random.uniform(-1, 1, 3)
        #     objects.append(Plane("Moving_Plane", origin, normal, size=0.15))

        #     # 添加一个运动的矩形
        #     center = np.random.uniform(-0.3, 0.3, 3)
        #     normal = np.random.uniform(-1, 1, 3)
        #     objects.append(Rectangle("Moving_Rect", center, normal,
        #                            width=0.1, height=0.08))

        #     # 更新场景
        scene_3d.update(objects)

        # 控制更新频率
        time.sleep(10)  # 大约30fps

    finally:
        scene_3d.close()  # 确保程序结束时正确关闭场景


if __name__ == "__main__":
    main()
