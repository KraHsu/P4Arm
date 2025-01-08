import numpy as np
from vispy import app, scene
from vispy.visuals.transforms import MatrixTransform
import threading

class Frame:
    def __init__(self, name, H):
        self.name = name
        self.H = H.astype(np.float32)
        
    def get_origin(self):
        return self.H[:3, 3].flatten()

    def create_axis_data(self, scale=1.0):
        origin = self.H[:3, 3]
        R = self.H[:3, :3]
        
        axes_endpoints = [
            origin + scale * R[:, 0],
            origin + scale * R[:, 1],
            origin + scale * R[:, 2]
        ]
        
        axes_data = []
        for endpoint in axes_endpoints:
            axes_data.append(origin)
            axes_data.append(endpoint)
            
        return np.array(axes_data, dtype=np.float32)

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
        self.x_labels = scene.Text(parent=self.view.scene, color='black', font_size=8)
        self.y_labels = scene.Text(parent=self.view.scene, color='black', font_size=8)
        self.z_labels = scene.Text(parent=self.view.scene, color='black', font_size=8)
        
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
        self.canvas = scene.SceneCanvas(keys='interactive', size=(800, 600), bgcolor='#ffffff')
        
        # 创建3D视图
        self.view = self.canvas.central_widget.add_view()
        self.view.camera = scene.TurntableCamera(up='z', fov=60)
        
        # 添加坐标轴和标签
        axis = scene.XYZAxis(parent=self.view.scene)
        
        # 添加带颜色的网格线
        grid = scene.GridLines(parent=self.view.scene, scale=(0.1, 0.1),
                             color=(0.5, 0.5, 0.5, 0.5))  # 灰色半透明
        
        # 添加垂直于x轴的网格
        grid_yz = scene.GridLines(parent=self.view.scene, scale=(0.1, 0.1),
                                color=(0.5, 0.5, 0.5, 0.3))
        grid_yz.transform = scene.transforms.MatrixTransform()
        grid_yz.transform.rotate(90, (0, 1, 0))
        
        # 添加垂直于y轴的网格
        grid_xz = scene.GridLines(parent=self.view.scene, scale=(0.1, 0.1),
                                color=(0.5, 0.5, 0.5, 0.3))
        grid_xz.transform = scene.transforms.MatrixTransform()
        grid_xz.transform.rotate(90, (1, 0, 0))
        
        # 添加网格刻度标签
        self._create_grid_labels()
        
        # 创建视觉对象
        self.lines = scene.visuals.Line(parent=self.view.scene, 
                                      width=2,
                                      connect='segments',
                                      antialias=True)
        
        self.origins = scene.visuals.Markers(parent=self.view.scene)
        self.labels = scene.visuals.Text(parent=self.view.scene,
                                       color='black',
                                       face='OpenSans',
                                       anchor_x='left',
                                       anchor_y='bottom')
        
        # 设置初始视图范围
        self.view.camera.set_range(x=[-0.5, 0.5], y=[-0.5, 0.5], z=[-0.5, 0.5])
        
        # 设置网格的GL状态
        grid.set_gl_state('translucent', depth_test=False)
        grid_yz.set_gl_state('translucent', depth_test=False)
        grid_xz.set_gl_state('translucent', depth_test=False)
        
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
        """更新场景中的所有坐标框架"""
        if not objs or not self._running:
            return
            
        all_lines = []
        all_colors = []
        origins = []
        labels = []
        label_pos = []
        
        for obj in objs:
            frame_lines = obj.create_axis_data(self.scale)
            all_lines.extend(frame_lines)
            all_colors.extend([[1,0,0,1]]*2)  # X轴红色
            all_colors.extend([[0,1,0,1]]*2)  # Y轴绿色
            all_colors.extend([[0,0,1,1]]*2)  # Z轴蓝色
            
            origin = obj.get_origin()
            origins.append(origin)
            labels.append(obj.name)
            label_pos.append(origin)
        
        def _update():
            if not self._running:
                return
            self.lines.set_data(pos=np.array(all_lines, dtype=np.float32),
                              color=np.array(all_colors, dtype=np.float32))
            
            self.origins.set_data(pos=np.array(origins, dtype=np.float32),
                                edge_color=None,
                                face_color=(0,0,0,1),
                                size=10)
            
            self.labels.text = labels
            self.labels.pos = np.array(label_pos, dtype=np.float32)
            self.canvas.update()
        
        app.Timer(connect=lambda _: _update(), iterations=1).start()

def random_transform_matrix():
    from scipy.spatial.transform import Rotation
    """生成一个合理的随机变换矩阵"""
    R = Rotation.random().as_matrix()
    t = np.random.uniform(-0.3, 0.3, 3)
    H = np.eye(4)
    H[:3, :3] = R
    H[:3, 3] = t
    return H

def main():
    scene_3d = Realtime3DScene()
    import time
    
    try:
        for i in range(30 * 10):
            frames = []
            for j in range(10):
                H = random_transform_matrix()
                frames.append(Frame(f"Frame_{i}_{j}", H))
            scene_3d.update(frames)
            time.sleep(0.03)
    finally:
        scene_3d.close()  # 确保程序结束时正确停止场景

# 使用示例
if __name__ == "__main__":
    main()