import numpy as np
import open3d as o3d
import os
from matplotlib import cm
from matplotlib.colors import Normalize

GREEN = np.array([0, 255, 0]) / 255.0
BLACK = np.array([0, 0, 0]) / 255.0
WHITE = np.array([255, 255, 255]) / 255.0
SPHERE_SIZE = 0.20

class Visualizer():
    def __init__(self):
        self.pause = False
        self.reset_view_point = True

        self.current = o3d.geometry.PointCloud()
        self.local_map = o3d.geometry.PointCloud()
        self.global_map = o3d.geometry.PointCloud()
        self.trajectory_points = []

        self.vis = o3d.visualization.VisualizerWithKeyCallback()
        self._register_key_callbacks()
        self._initialize_visualizer()  # ✅ Enable live rendering
        self.vis_counter = 0

    def _initialize_visualizer(self):
        self.vis.create_window(window_name="MAD-ICP", width=1280, height=720, visible=True)
        self.vis.add_geometry(self.current)
        self.vis.add_geometry(self.local_map)
        self.vis.get_render_option().background_color = BLACK
        self.vis.get_render_option().point_size = 1.5
        print("Visualizer commands:\n"
              "\t[SPACE] to pause/start\n"
              "\t    [X] to center the viewpoint\n")

    def _register_key_callback(self, key, callback):
        self.vis.register_key_callback(ord(key), callback)

    def _register_key_callbacks(self):
        self._register_key_callback(" ", self._pause)
        self._register_key_callback("X", self._reset_view_point)

    def _pause(self, vis):
        self.pause = not self.pause

    def _reset_view_point(self, vis):
        self.reset_view_point = True

    def update(self, current, local_map, pose, kf):
        self._update_geometries(current, local_map, pose, kf)
        if len(self.trajectory_points) > 0:
            view_ctl = self.vis.get_view_control()
            view_ctl.set_lookat(self.trajectory_points[-1])
            view_ctl.set_front([0, 0, -1])
            view_ctl.set_up([0, -1, 0])
            view_ctl.set_zoom(0.5)
            
            self.vis.poll_events()
            self.vis.update_renderer()
            
            # ✅ Save current frame as image
            os.makedirs("frames", exist_ok=True)
            self.vis.capture_screen_image(f"frames/frame_{self.vis_counter:04d}.png", do_render=True)
            self.vis_counter += 1
            
            # 👇 Pause loop if paused
            while self.pause:
                self.vis.poll_events()
                self.vis.update_renderer()

    def _update_geometries(self, current, local_map, pose, kf):
        self.current.points = o3d.utility.Vector3dVector(current)
        self.current.paint_uniform_color(WHITE)
        self.vis.update_geometry(self.current)

        if local_map is not None:
            # Transform local map to global using pose
            local_np = np.array(local_map)
            ones = np.ones((local_np.shape[0], 1))
            local_hom = np.hstack((local_np, ones))  # Nx4
            transformed = (pose @ local_hom.T).T[:, :3]

            # Accumulate in global map
            temp_pcd = o3d.geometry.PointCloud()
            temp_pcd.points = o3d.utility.Vector3dVector(transformed)
            self.global_map.points.extend(temp_pcd.points)

            # Visualization: show local map (colored)
            self.local_map.points = o3d.utility.Vector3dVector(transformed)
            colors = cm.plasma(Normalize()(transformed[:, 2]))[:, :3]
            self.local_map.colors = o3d.utility.Vector3dVector(colors)
            self.vis.update_geometry(self.local_map)

            # Optional PCD saving
            os.makedirs("pcd_final", exist_ok=True)
            o3d.io.write_point_cloud(f"pcd_final/local_map_{self.vis_counter:04d}.pcd", self.local_map)

        # Record current pose in world coordinates for trajectory
        current_position = (pose @ np.array([0, 0, 0, 1]).T)[:3]
        self.trajectory_points.append(current_position.tolist())

        if len(self.trajectory_points) > 1:
            points = np.array([self.trajectory_points[-2], self.trajectory_points[-1]])
            trajectory_line = o3d.geometry.LineSet()
            trajectory_line.points = o3d.utility.Vector3dVector(points)
            trajectory_line.lines = o3d.utility.Vector2iVector([[0, 1]])
            trajectory_line.paint_uniform_color(GREEN)
            self.vis.add_geometry(trajectory_line)
            self.vis.update_geometry(trajectory_line)

        if kf is not None:
            sphere = o3d.geometry.TriangleMesh.create_sphere(SPHERE_SIZE)
            sphere.paint_uniform_color(GREEN)
            sphere.transform(pose @ kf)  # Optional: use pose @ kf if kf is relative
            self.vis.add_geometry(sphere)
            self.vis.update_geometry(sphere)

        if self.reset_view_point:
            self.vis.reset_view_point(True)
            self.reset_view_point = False
