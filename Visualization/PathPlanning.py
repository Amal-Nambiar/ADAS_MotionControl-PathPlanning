import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class LocalMockVehicle:
    def __init__(self, history, state):
        self.history = history
        self.state = state

class PathVisualizer:
    def __init__(self, map_instance, vehicle_config, plotter_instance, figsize=(10, 10), camera_zoom=False):
        self.map_instance = map_instance
        self.cfg = vehicle_config
        self.plotter = plotter_instance
        self.figsize = figsize
        self.camera_zoom = camera_zoom

    def animate_pipeline(self, start_pose, goal_pose, reference_path, search_history, 
                         tracking_history, state_snapshots, target_points, search_windows, algo_profile=None,
                         title_name="Autonomous Navigation Framework", save_gif=False, gif_name="simulation.gif"):
        
        num_search_frames = len(search_history)
        num_pause_frames = 20  
        num_drive_frames = len(state_snapshots)
        total_frames = num_search_frames + num_pause_frames + num_drive_frames

        fig, ax = plt.subplots(figsize=self.figsize)
        map_width = self.map_instance.grid_map.width
        map_height = self.map_instance.grid_map.height

        prof = algo_profile if algo_profile is not None else {
             "name": "Global Planner", 
             "compute_time_ms": 0.0, 
             "nodes_explored": 0, 
             "path_length": 0
         }

        open_node_coords = []
        close_node_coords = []
        primitive_branches = []
        frenet_mode = False  

        for snap in search_history:           
            if isinstance(snap["open"], dict):
                open_x = [node.x for node in snap["open"].values()]
                open_y = [node.y for node in snap["open"].values()]
                open_node_coords.append((open_x, open_y))
                
                close_x = [node.x for node in snap["close"].values()]
                close_y = [node.y for node in snap["close"].values()]
                close_node_coords.append((close_x, close_y))
            else:
                frenet_mode = True
                open_node_coords.append(snap["open"])
                close_node_coords.append(snap["close"])

            
            # Store continuous path primitives if they exist (State Lattice mode)
            if "primitives" in snap:
                primitive_branches.append(snap["primitives"])

        def update_frame(frame, target_ax):
            target_ax.clear()  
            target_ax.imshow(self.map_instance.grid_map.grid, origin="lower", cmap="Greys", alpha=0.6)
            target_ax.grid(True, linestyle=":", alpha=0.3)
            
            target_ax.scatter(start_pose[0], start_pose[1], color='blue', marker='o', s=150, zorder=10, label='Start')
            target_ax.scatter(goal_pose[0], goal_pose[1], color='green', marker='X', s=150, zorder=10, label='Goal')

            # ------------------------------------------------------------------
            # STAGE 1: GLOBAL SEARCH STATE SPACE (Finding the Goal)
            # ------------------------------------------------------------------
            if frame < num_search_frames:
                target_ax.set_title(f"1. Global Planning: {prof['name']} (Step {frame})", fontsize=12, fontweight='bold')
                
                # Check if we are running in State Lattice mode with continuous curves
                if primitive_branches:
                    current_paths = primitive_branches[frame]
                    for path_coords in current_paths:
                        tx = [p[0] for p in path_coords]
                        ty = [p[1] for p in path_coords]
                        target_ax.plot(tx, ty, color='black', linewidth=0.5, alpha=0.25)

                elif frenet_mode:
                    raw_frame_idx = frame // 4
                    valid_paths = open_node_coords[raw_frame_idx]
                    blocked_paths = close_node_coords[raw_frame_idx]

                    for path in blocked_paths:
                        target_ax.plot(path.x, path.y, color = 'red', alpha = 0.2,linestyle=':', linewidth=0.6 )    

                    for path in valid_paths:           
                        target_ax.plot(path.x, path.y, color = 'green', alpha = 0.4, linewidth=0.8)
                
                else:
                    # Fallback to standard Hybrid A* discrete point-clouds
                    ox, oy = open_node_coords[frame]
                    cx, cy = close_node_coords[frame]
                    if ox: target_ax.scatter(ox, oy, marker='.', color='cyan', alpha=0.4, edgecolors='none')
                    if cx: target_ax.scatter(cx, cy, marker='.', color='red', alpha=0.2, edgecolors='none')
                    
                mock_vehicle = LocalMockVehicle(tracking_history[0], state_snapshots[0])
                self.plotter.draw_body(target_ax, mock_vehicle.state)
                self.plotter.draw_wheels(target_ax, mock_vehicle.state)
                
                search_text = (
                    f"Algorithm: {prof['name']}\n"
                    f"Status   : Expanding Primitives...\n"
                    f"Step     : {frame} / {num_search_frames}"
                )
                target_ax.text(0.03, 0.97, search_text, transform=target_ax.transAxes, fontsize=11,
                               fontweight='bold', verticalalignment='top', 
                               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightcyan', alpha=0.85))

                target_ax.set_xlim(0, map_width); target_ax.set_ylim(0, map_height)

            # ------------------------------------------------------------------
            # STAGE 2: PATH RETRIEVAL PAUSE (Goal Identified Successfully)
            # ------------------------------------------------------------------
            elif frame < (num_search_frames + num_pause_frames):
                target_ax.set_title("2. Shortest Global Route Computed Successfully!", fontsize=12, fontweight='bold')
                target_ax.plot(reference_path.x, reference_path.y, color='magenta', linewidth=3, label="Planned Route", zorder=8)
                
                mock_vehicle = LocalMockVehicle(tracking_history[0], state_snapshots[0])
                self.plotter.draw_body(target_ax, mock_vehicle.state)
                self.plotter.draw_wheels(target_ax, mock_vehicle.state)
                
                benchmark_text = (
                    f"▼ {prof['name']} Results:\n"
                    f"• Compute Time  : {prof['compute_time_ms']:.2f} ms\n"
                    f"• Nodes Visited : {prof['nodes_explored']} iterations\n"
                    f"• Route Length  : {prof['path_length']} coordinates"
                )
                target_ax.text(0.03, 0.97, benchmark_text, transform=target_ax.transAxes, fontsize=11,
                               fontweight='bold', verticalalignment='top', 
                               bbox=dict(boxstyle='round,pad=0.5', facecolor='lightgreen', alpha=0.9))

                target_ax.set_xlim(0, map_width); target_ax.set_ylim(0, map_height)

            # ------------------------------------------------------------------
            # STAGE 3: KINEMATIC SIMULATION TRACKING (Vehicle Moving)
            # ------------------------------------------------------------------
            else:
                v_frame = frame - num_search_frames - num_pause_frames
                safe_v_frame = min(v_frame, len(state_snapshots) - 1)

                target_ax.set_title(f"3. Autonomous Tracking: Stanley Control (Frame {safe_v_frame})", fontsize=12, fontweight='bold')
                target_ax.plot(reference_path.x, reference_path.y, "--", color='magenta', linewidth=2, label="Reference Path", alpha=0.7)
                
                wx, wy = search_windows[safe_v_frame]
                target_ax.plot(wx, wy, 'o', color='purple', markersize=4, label='Search Window', alpha=0.5)
                tx, ty = target_points[safe_v_frame]
                target_ax.plot(tx, ty, 'ro', label='Target Point', markersize=8)

                mock_vehicle = LocalMockVehicle(tracking_history[safe_v_frame], state_snapshots[safe_v_frame])
                self.plotter.draw_trajectory(target_ax, mock_vehicle.history)
                self.plotter.draw_body(target_ax, mock_vehicle.state)
                self.plotter.draw_heading(target_ax, mock_vehicle.state)
                self.plotter.draw_wheels(target_ax, mock_vehicle.state)

                current_state = state_snapshots[safe_v_frame]
                telemetry_text = (
                    f"Tracker       : Stanley Steering\n"
                    f"Current Speed : {current_state.v*3.6:.2f} kmph\n"
                    f"Steer Angle   : {np.degrees(current_state.delta):.1f}°"
                )
                target_ax.text(0.03, 0.97, telemetry_text, transform=target_ax.transAxes, fontsize=11,
                               fontweight='bold', verticalalignment='top', 
                               bbox=dict(boxstyle='round,pad=0.5', facecolor='white', alpha=0.8))

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
            target_ax.legend(loc="upper right")

        anim = FuncAnimation(fig, update_frame, fargs=(ax,), frames=total_frames, interval=80, repeat=False)

        if save_gif:
            print(f"Compiling pipeline animation and saving to {gif_name}...")
            anim.save(gif_name, writer='pillow', fps=12)
            print("GIF Save Completed successfully.")
        else:
            plt.show()
