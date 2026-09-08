import os
import sys
import copy
import numpy as np
import time

current_dir = os.getcwd()
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path: sys.path.append(parent_dir)

# 1. IMPORT MODULAR APP LIBRARIES
from utils.Different_map import Map
from utils.Smooth_Curve import PathGeneration
from utils.StateLattice.node import *
from utils.StateLattice.MotionPrimitive import MotionPrimitiveGenerator
from utils.Hybrid.motion_model import *
from utils.Hybrid.collision_checker import *
from utils.Hybrid.holonomic_heuristic import *
from utils.reference_path import *
from utils.Hybrid.reeds_shepp_connector import ReedsSheppConnector

from Visualization.PathPlanning import PathVisualizer  
from Vehicle.Vehicle_config import VehicleConfig
from Vehicle.vehicle_state import VehicleState
from Vehicle.control_input import ControlInput
from Vehicle.vehicle import Vehicle
from Controllers.pid_speed_controller import PIDSpeedController
from Controllers.Stanley_controller import StanleyController
from Planning.StateLattice import *
from VehiclePlotter import VehiclePlotter

# Spline math geometry wrapper path tracking helper
class ComputedReferencePath:
    def __init__(self, x_coords, y_coords):
        self.x, self.y = np.array(x_coords, dtype=float), np.array(y_coords, dtype=float)
        path_yaw = np.arctan2(np.diff(self.y), np.diff(self.x))
        self.yaw = np.append(path_yaw, path_yaw[-1]) if len(path_yaw) > 0 else np.array([0.0])

# 2. ENVIRONMENT INITIALIZATION
diff_map = Map()
diff_map.map_5_maze(width=100, height=70)  # Standard structure
grid_map = diff_map.grid_map

# Using clear configurations known to solve successfully
start_pose = (10, 5, np.deg2rad(0))
goal_pose  = (85, 50, np.deg2rad(-45))

cfg = VehicleConfig()
plotter = VehiclePlotter(cfg)
motion = MotionModel(cfg.wheelbase)
collision_checker = CollisionChecker(grid_map=grid_map, Vehicle_cfg=cfg)
reeds_Shepp = ReedsSheppConnector(collision_checker, cfg)

# 3. GLOBAL PLANNING: STATE LATTICE EXPLORATION
print("Generating holonomic heuristic map targeting destination...")
hh = HolonomicHeuristic(grid_map)
hmap = hh.build(goal_pose=(goal_pose[0], goal_pose[1]))

print("Generating motion primitive library...")
MPG = MotionPrimitiveGenerator(motion, hmap)
library = MPG.generate_library(max_steer=0.5)

lattice_planner = StateLatticePlanner(grid_map, library, collision_checker, MPG, reeds_connector=reeds_Shepp)

print("Running State Lattice Path Finder...")
start_cpu = time.perf_counter()
success = lattice_planner.plan(start_pose, goal_pose, hmap)
end_cpu = time.perf_counter()

if not success:
    print("Error: State Lattice failed to resolve coordinates. Aborting control phase.")
    sys.exit()

rx, ry = lattice_planner.reconstruct_path()
ref_path    = ReferencePath(rx, ry)
ref_curv    = ref_path.compute_curvature()

algo_profile = {
    "name": "State Lattice Search",
    "compute_time_ms": (end_cpu - start_cpu) * 1000.0,
    "nodes_explored": len(lattice_planner.search_history),
    "path_length": len(rx)
}
reference_path = ComputedReferencePath(rx, ry)

# 4. KINEMATIC SIMULATION PIPELINES (Stanley + PID tracking)
state = VehicleState()
# Initialize vehicle state exactly matching the starting element of our solved lattice path
state.x, state.y, state.v, state.delta = rx[0], ry[0], 0.0, 0.0
state.yaw = reference_path.yaw[0]
vehicle_run = Vehicle(cfg, state)

st_controller = StanleyController(wheelbase=cfg.wheelbase, k=1.2) 
pid_speed = PIDSpeedController(kp=1.2, ki=0.05, kd=0.5)

history_snapshots, state_snapshots, target_points, search_windows = [], [], [], []

print("Engaging Stanley and PID Tracking Control Loops...")
for _ in range(1200):
    lat_acclr = 1.5
    safe_curv = np.where(np.abs(ref_curv) < 1e-5, 1e-5, np.abs(ref_curv))
    tgt_speed_profile = np.sqrt(lat_acclr / safe_curv)
    MAX_SPEED_MPS = 5.55
    MIN_SPEED_MPS = 1.38
    tgt_speed_profile = np.clip(tgt_speed_profile, a_min=MIN_SPEED_MPS, a_max=MAX_SPEED_MPS)
    
    desired_steer, metrics = st_controller.compute_steering(vehicle_run.state, reference_path)
    look_ahead_idx = min(metrics["target_idx"] + 8, len(tgt_speed_profile) - 1)
    current_target_speed = tgt_speed_profile[look_ahead_idx]
    accel = pid_speed.compute_control(target_speed=current_target_speed, current_speed=vehicle_run.state.v, dt=0.1)   

    if metrics["target_idx"] >= len(reference_path.x) - 3: 
        print("Vehicle arrived at the terminal destination threshold successfully!")
        break
    
    control = ControlInput(accel=accel, steer_rate=4.0 * (desired_steer - vehicle_run.state.delta))    
    vehicle_run.step(control, dt=0.1)
    
    target_points.append(metrics["target_point"])
    search_windows.append((metrics["window_x"], metrics["window_y"]))    
    history_snapshots.append(copy.deepcopy(vehicle_run.history))
    state_snapshots.append(copy.deepcopy(vehicle_run.state))

# 5. EXECUTE MULTI-STAGE COMPOSITE GRAPHICS ANIMATION
print("Launching Composite Pipeline Animation...")
vis = PathVisualizer(diff_map, cfg, plotter, camera_zoom=False)
vis.animate_pipeline(
    start_pose=start_pose[:2], # Visualizer expects (x, y) 
    goal_pose=goal_pose[:2], 
    reference_path=reference_path, 
    search_history=lattice_planner.search_history,   
    tracking_history=history_snapshots,      
    state_snapshots=state_snapshots,          
    target_points=target_points,              
    search_windows=search_windows,            
    algo_profile=algo_profile,
    title_name="State Lattice Integrated Autonomy Simulation",
    save_gif=False,                            
    gif_name="State_Lattice_Stanley.gif"
)
