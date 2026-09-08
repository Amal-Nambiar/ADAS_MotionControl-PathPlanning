import numpy as np
import sys
import os
current_dir = os.getcwd()
parent_dir = os.path.abspath(os.path.join(current_dir, ".."))

if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.geometry                    import *
from utils.reference_path              import *
from utils.path_generation             import *
from Controllers.pid_speed_controller  import *

class VehiclePlotter:

    def __init__(self, cfg, camera_zoom = True):
        self.cfg = cfg
        self.camera_zoom = camera_zoom

    def get_body_polygon(self, state):
        front  = self.cfg.wheelbase + self.cfg.front_overhang
        rear   = self.cfg.rear_overhang
        half_w = self.cfg.width / 2

        body = np.array([
            [front, -half_w],  # Front Right
            [front, half_w],   # Front Left
            [-rear, half_w],   # Rear Left
            [-rear, -half_w],  # Rear Right
            [front, -half_w]   # Back to Front Right 
        ])

        R = rotation_matrix(state.yaw)
        body = body @ R.T
        body[:, 0] += state.x
        body[:, 1] += state.y
        return body

    def draw_body(self, ax, state):
        body = self.get_body_polygon(state)
        ax.plot(
            body[:, 0],
            body[:, 1],
            color='black',
            linewidth=2
        )

    def draw_heading(self, ax, state):
        ax.arrow(state.x, state.y, np.cos(state.yaw), np.sin(state.yaw), width = 0.05, color = 'red')

    def draw_trajectory(self, ax, history):
        ax.plot(history["x"], history["y"], color = "blue")

    def create_wheel_polygon(self):
        wl = self.cfg.wheel_length
        ww = self.cfg.wheel_width

        return np.array([
            [wl/2, -ww/2],
            [wl/2, ww/2],
            [-wl/2, ww/2],
            [-wl/2, -ww/2],
            [wl/2, -ww/2]
        ])

    def draw_wheel(self, ax, center_x, center_y, wheel_yaw):
        wheel = self.create_wheel_polygon()
        wheel = transform_polygon(wheel, center_x, center_y, wheel_yaw)
        ax.plot(wheel[:, 0], wheel[:, 1], color="black", linewidth=1.5)

    def draw_wheels(self, ax, state):
        half_track = self.cfg.width / 2
        wheel_centers = {
            "RL": (0.0, half_track),
            "RR": (0.0, -half_track),
            "FL": (self.cfg.wheelbase, half_track),
            "FR": (self.cfg.wheelbase, -half_track)
        }
        
        R_car = rotation_matrix(state.yaw)

        steering_angle = getattr(state, 'steering', 
                         getattr(state, 'delta', 
                         getattr(state, 'steer', 0.0)))        
        
        for name, (cx, cy) in wheel_centers.items():
            local_pos = np.array([cx, cy])
            global_pos = R_car @ local_pos + np.array([state.x, state.y])
            
            if name in ["FL", "FR"]:
                global_wheel_yaw = state.yaw + steering_angle
            else:
                global_wheel_yaw = state.yaw
                
            self.draw_wheel(ax, global_pos[0], global_pos[1], global_wheel_yaw)

    def draw_path(self, ax, path):
        ax.plot(path.x, path.y, "--", color = "green", linewidth = 2, label  = "Reference Path")

    def draw(self, ax, vehicle, path = None):
        if path is not None:
            self.draw_path(ax, path)
            
        self.draw_trajectory(ax, vehicle.history)
        self.draw_body(ax, vehicle.state)
        self.draw_heading(ax, vehicle.state)
        self.draw_wheels(ax, vehicle.state)

    def draw_target_point(self, ax, x, y):
        ax.plot(x, y, 'ro', label = 'Target', markersize = 8)

    def animate(self, frame, ax, path, search_windows_history, closest_points_history, target_points, state_snapshots, history_snapshots, title_name="Path Tracking"):
        ax.clear()
        
        ax.set_aspect("equal", adjustable="datalim")
        ax.grid(True)
        
        self.draw_path(ax, path)     
        wx, wy = search_windows_history[frame]
        ax.plot(wx, wy, 'o', color = 'magenta', markersize = 4, label = 'Window', alpha = 0.6)
        cx, cy = closest_points_history[frame]
        ax.plot(cx, cy, 's', color = 'cyan', markersize = 6, label ='Closest Point')
        tx, ty = target_points[frame]
        self.draw_target_point(ax, tx, ty)
        
        mock_vehicle = MockVehicle(history_snapshots[frame], state_snapshots[frame])
        self.draw(ax, mock_vehicle)
        
        current_state = state_snapshots[frame]
        ax.set_title(f"{title_name} (Frame {frame})", fontsize=14, fontweight='bold')
        
        steering_rad = getattr(current_state, 'delta', getattr(current_state, 'steering', 0.0))
        steering_deg = np.degrees(steering_rad)
        telemetry_text = (
            f"Target_Speed: {20:.2f} kmph \n"
            f"Current_Speed: {current_state.v*3.6:.2f} kmph\n"
            f"Steer Angle: {steering_deg:.1f}°"
        )
        
        ax.text(
            0.02, 0.95, telemetry_text,
            transform=ax.transAxes, 
            fontsize=12, fontweight='bold',
            verticalalignment='top', 
            bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8, edgecolor='gray')
        )                

        if self.camera_zoom:
            ax.set_aspect("equal", adjustable="box")
            
            cam_window = 20  # View radius (meters) around the car
            ax.set_xlim(current_state.x - cam_window, current_state.x + cam_window)
            ax.set_ylim(current_state.y - cam_window, current_state.y + cam_window)   
        else:
            ax.set_aspect("equal", adjustable="datalim")
            ax.set_autoscale_on(True)
            ax.relim()
            ax.autoscale_view(True, True, True)


class MockVehicle:
    def __init__(self, history, state):
        self.history = history
        self.state = state        
