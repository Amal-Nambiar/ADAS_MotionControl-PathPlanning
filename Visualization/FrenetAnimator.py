import numpy as np
import matplotlib.pyplot as plt
import os
import sys
from matplotlib.animation import FuncAnimation

current_dir = os.getcwd()
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path: 
    sys.path.append(parent_dir)


from utils.Frenet.reference_line import *
from utils.Frenet.frenet_converter import *
from utils.Frenet.frenet_trajectory import *
from utils.Frenet.Dynamic_Obstacle import *
from utils.Frenet.FrenetSnapshot import *
from utils.Smooth_Curve import *
from utils.path_generation import *
from utils.Frenet.FrenetRoad import *
from Planning.FrenetOptimalPlanner import *

    
from Frenet_Simulation import *
from Vehicle.Vehicle_config import VehicleConfig
from Vehicle.vehicle_state import VehicleState
from Vehicle.control_input import ControlInput
from Vehicle.vehicle import Vehicle
from Planning.FrenetOptimalPlanner import * 


path = generate_racetrack_path(length=60, width=30, n_points=1000)
x_ref  = path.x
y_ref  = path.y
ref    = reference_line(x_ref, y_ref)
frenet = FrenetConverter(ref) 
road = FrenetRoad(frenet,lane_width=3.5, n_lanes=5)
lanes = road.create_lanes()

cfg = VehicleConfig()
plotter = VehiclePlotter(cfg)
state = VehicleState()
state.x = path.x[0]
state.y = path.y[0]
state.yaw = ref.yaw[0]
state.v = 0.0
vehicle_run = Vehicle(cfg, state)
ego_s, ego_d = frenet.cartesian_to_sd(state.x,state.y)
obstacle_s = ego_s + 80
obstacle_d = 0.0
obstacle_s1 = ego_s + 150
obstacle_d1 = -3.5
obstacle_s2 = ego_s + 120
obstacle_d2 = 3.5
obstacle_s3 = ego_s + 40
obstacle_d3 = 7.0
obstacle = DynamicObstacle(obstacle_s,obstacle_d,3)
obstacle1 = DynamicObstacle(obstacle_s1,obstacle_d1,3)
obstacle2 = DynamicObstacle(obstacle_s2,obstacle_d2,5)
obstacle3 = DynamicObstacle(obstacle_s3,obstacle_d3,2)
all_obstacles = [obstacle, obstacle1, obstacle2, obstacle3]
planner = FrenetPlanner(frenet,all_obstacles,path.x[0],path.y[0])
stanley = StanleyController(wheelbase=cfg.wheelbase, k=0.4) 
pid = PIDSpeedController(kp=1.2, ki=0.05, kd=0.5)
print("START OF FILE")
class FrenetAnimator:

    def __init__(
            self,
            snapshots,
            lanes,
            plotter):

        self.snapshots = snapshots
        self.lanes = lanes
        self.plotter = plotter

    def animate(self):
        print("Creating Simulation")
        fig, ax = plt.subplots(
            figsize=(12,8)
        )

        camera_size = 60
        def update(frame):
            print("rendering Frame", frame)
            ax.clear()
            snap = self.snapshots[frame]

            # ----------------
            # Draw Lanes
            # ----------------

            for lane_name,(lx,ly) in self.lanes.items():
                ax.plot( lx, ly, color="black", linewidth=1)

            # ----------------
            # Candidate Paths
            # ----------------
            for px, py, is_valid in snap.candidate_paths:
                if is_valid :
                    ax.plot( px, py, color="gray", alpha=0.6, linewidth =1)
                else:
                    ax.plot( px, py, color="red", alpha=0.6, linewidth =1)

            # ----------------
            # Best Path
            # ----------------
            ax.plot( snap.best_x, snap.best_y, color="green", linewidth=4)
            # ----------------
            # Obstacle
            # ----------------
            ax.scatter( snap.obstacles_x, snap.obstacles_y, color="red", s=150, zorder = 3)
            # ----------------
            # Vehicle
            # ----------------
            self.plotter.draw_body( ax, snap.vehicle_state)
            self.plotter.draw_wheels(ax, snap.vehicle_state)
            self.plotter.draw_heading( ax, snap.vehicle_state)
            vx = snap.vehicle_state.x
            vy = snap.vehicle_state.y
            ax.set_xlim(vx - camera_size, vx + camera_size)
            ax.set_ylim(vy - camera_size, vy + camera_size)
            ax.set_aspect("equal")
            ax.grid(True)

        ani = FuncAnimation( fig, update, frames=len(self.snapshots), interval=40, repeat=True)
        self.ani = ani
        plt.show(block=True)
sim = FrenetSimulation(vehicle_run,  planner, all_obstacles, stanley, pid)

snapshots = sim.run()
print("Snapshots:", len(snapshots))
animator = FrenetAnimator(snapshots, lanes, plotter)
print("Starting Animation")
animator.animate()        