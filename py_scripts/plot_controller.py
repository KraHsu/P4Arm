import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D


class Frame:
    def __init__(self, name, H):
        self.name = name
        self.H = H

    def get_origin(self):
        return self.H[:3, 3].flatten()

    def draw(self, ax, scale=1.0):
        world_RT = self.H

        origin = world_RT[:3, 3]

        world_R = world_RT[:3, :3]

        x_axis = origin + scale * world_R[:, 0]
        y_axis = origin + scale * world_R[:, 1]
        z_axis = origin + scale * world_R[:, 2]

        ax.plot(
            [origin[0], x_axis[0]],
            [origin[1], x_axis[1]],
            [origin[2], x_axis[2]],
            "r",
            label=f"{self.name}_x",
        )
        ax.plot(
            [origin[0], y_axis[0]],
            [origin[1], y_axis[1]],
            [origin[2], y_axis[2]],
            "g",
            label=f"{self.name}_y",
        )
        ax.plot(
            [origin[0], z_axis[0]],
            [origin[1], z_axis[1]],
            [origin[2], z_axis[2]],
            "b",
            label=f"{self.name}_z",
        )

        ax.scatter(origin[0], origin[1], origin[2], c="k", s=50)

        ax.text(origin[0], origin[1], origin[2], f" {self.name}", color="k")


class Realtime3DScene:
    def __init__(self):
        # 初始化图形
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_xlim(-10, 10)
        self.ax.set_ylim(-10, 10)
        self.ax.set_zlim(-10, 10)

        self.ax.set_xlabel("X Axis")
        self.ax.set_ylabel("Y Axis")
        self.ax.set_zlabel("Z Axis")

        self.scale = 1

        plt.ion()  # 开启交互模式
        plt.show()

    def update(self, objs, t=0.01):
        # 更新图形内容
        self.ax.cla()

        if len(objs):
            points = np.array([obj.get_origin() for obj in objs])

            x = points[:, 0]
            y = points[:, 1]
            z = points[:, 2]

            self.ax.set_xlim(np.min(x) - 1, np.max(x) + 1)
            self.ax.set_ylim(np.min(y) - 1, np.max(y) + 1)
            self.ax.set_zlim(np.min(z) - 1, np.max(z) + 1)

            self.scale = (
                np.min(
                    [
                        np.max(x) - np.min(x),
                        np.max(y) - np.min(y),
                        np.max(z) - np.min(z),
                    ]
                )
                / 5
                + 0.4
            )

        for obj in objs:
            obj.draw(
                self.ax,
                self.scale,
            )

        self.ax.set_xlabel("X Axis")
        self.ax.set_ylabel("Y Axis")
        self.ax.set_zlabel("Z Axis")

        plt.pause(t)

    def close(self):
        # 关闭图形
        plt.ioff()
        plt.close()


if __name__ == "__main__":
    import numpy as np

    scatter_plot = Realtime3DScene()
    for i in range(100):
        objs = []
        for j in range(100):
            H = np.random.rand(4, 4)
            objs.append(Frame(f"{j}", H))
        scatter_plot.update(objs)
    scatter_plot.close()
