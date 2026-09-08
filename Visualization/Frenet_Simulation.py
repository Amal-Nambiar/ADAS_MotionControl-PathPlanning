import copy
import numpy as np
import os
import sys
current_dir = os.getcwd()
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))
if parent_dir not in sys.path: 
    sys.path.append(parent_dir)

from Visualization.PathPlanning import PathVisualizer  
from Planning.Dijkstra_heappq import Dijkstra
from Planning.AStar import AStar
from Vehicle.Vehicle_config import VehicleConfig
from Vehicle.vehicle_state import VehicleState
from Vehicle.control_input import ControlInput
from Vehicle.vehicle import Vehicle
from Controllers.pid_speed_controller import PIDSpeedController
from Controllers.Stanley_controller import StanleyController

from utils.Frenet.reference_line import *
from utils.Frenet.frenet_converter import *
from utils.Frenet.frenet_trajectory import *
from utils.Frenet.Dynamic_Obstacle import *
from utils.Smooth_Curve import *
from utils.path_generation import *
from utils.Frenet.FrenetRoad import *
from utils.Frenet.FrenetSnapshot import *
from Planning.FrenetOptimalPlanner import *


from Visualization.VehiclePlotter import *

state = VehicleState()
cfg = VehicleConfig()
plotter = VehiclePlotter(cfg)
state = VehicleState()
vehicle_run = Vehicle(cfg, state)
class FrenetSimulation:

    def __init__(
            self,
            vehicle,
            planner,
            obstacles,
            stanley,
            pid):

        self.vehicle = vehicle
        self.planner = planner
        self.obstacles = obstacles

        self.stanley = stanley
        self.pid = pid

        self.snapshots = []

    def run(
            self,
            simulation_time=60.0,
            dt=0.1,
            target_speed=5.0):
        
        print("Entered run()")

        steps = int(
            simulation_time / dt
        )

        for step in range(steps):
            if step % 10 == 0:
                print("Step:", step)            

            current_time = (
                step * dt
            )

            # --------------------------------
            # Dynamic obstacles
            # --------------------------------
            obs_x_list = []
            obs_y_list = []

            for obs in self.obstacles:
                obs_s, obs_d = obs.predict(current_time)
                obs_s = (obs_s % self.planner.converter.reference_line.s[-1])
                ox, oy = self.planner.converter.frenet_to_cartesian(obs_s, obs_d)
                obs_x_list.append(ox)
                obs_y_list.append(oy)
            # --------------------------------
            # Frenet Planner
            # --------------------------------

            best_path,candidate_trajectories = ( self.planner.plan
                                                ( self.vehicle.state.x, 
                                                  self.vehicle.state.y, 
                                                  self.vehicle.state.yaw,
                                                  self.vehicle.state.v
                                                  )
                                                )

            if best_path is None:

                print(
                    "No valid path found"
                )

                break

            # --------------------------------
            # Stanley Path
            # --------------------------------

            reference_path = (
                ReferencePath(
                    np.array(
                        best_path.x
                    ),
                    np.array(
                        best_path.y
                    )
                )
            )

            desired_steer, metrics = (
                self.stanley.compute_steering(
                    self.vehicle.state,
                    reference_path
                )
            )

            # --------------------------------
            # PID Speed
            # --------------------------------

            accel = (
                self.pid.compute_control(
                    target_speed,
                    self.vehicle.state.v,
                    dt
                )
            )

            steer_rate = (
                desired_steer
                -
                self.vehicle.state.delta
            ) * 4.0

            control = ControlInput(
                accel=accel,
                steer_rate=steer_rate
            )

            self.vehicle.step(
                control,
                dt
            )

            # --------------------------------
            # Snapshot
            # --------------------------------

            candidate_paths = [

                (
                    traj.x,
                    traj.y,
                    traj.valid
                )

                for traj
                in candidate_trajectories
            ]


            snap = FrenetSnapshot(

                vehicle_state=
                    copy.deepcopy(
                        self.vehicle.state
                    ),

                obstacles_x=
                    obs_x_list,

                obstacles_y=
                    obs_y_list,

                best_x=
                    list(best_path.x),

                best_y=
                    list(best_path.y),

                candidate_paths=
                    candidate_paths
            )

            self.snapshots.append(
                snap
            )
        return self.snapshots