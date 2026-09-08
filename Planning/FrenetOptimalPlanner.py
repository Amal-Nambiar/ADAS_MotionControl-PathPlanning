import numpy as np
from utils.Frenet.frenet_trajectory import FrenetTrajectory
from utils.Smooth_Curve import (QuinticPolynomial, QuarticPolynomial)
class FrenetPlanner:

    def __init__(self, converter, obstacles, current_s, current_d):

        self.converter = converter
        self.obstacles  = obstacles
        self.current_s = current_s
        self.current_d = current_d
        # self.prediction_items = [3, 5, 7, 10]
        self.prediction_items = [7]
        self.d_targets = [ -7.0, -3.5,  0.0, 3.5, 7.0]
        # self.d_targets = [0]
        # self.v_targets = [0.0, 1.0, 2.5, 5.0]
        self.v_targets = [3, 5, 7]
        self.safety_radius = 1.5
        self.collision_step_skip = 5 

    def compute_ttc(self, gap, ego_speed, obstacle_speed):
        relative_velocity = (ego_speed  - obstacle_speed)
        if relative_velocity <= 0:
            return np.inf
        return (gap /relative_velocity )

    def plan(self, current_x, current_y, current_yaw, current_v):
        candidate_trajectories = []
        for target_d in self.d_targets:
            for target_v in self.v_targets:
                for T in self.prediction_items:
                    traj = FrenetTrajectory()
                    t_steps = np.linspace( 0, T, 100)
                    self.current_s, self.current_d = (self.converter.cartesian_to_sd(current_x, current_y)                    )
                    lat_poly = QuinticPolynomial(xs=self.current_d, vxs=0.0, axs=0.0, xe=target_d, vxe=0.0, axe=0.0, T=T)
                    lon_poly = QuarticPolynomial(xs=self.current_s, vxs=current_v, axs=0.0, vxe=target_v, axe=0.0, T=T)
                    traj.t = list(t_steps)
                    traj.s = [lon_poly.calc_point(tt) for tt in t_steps ]
                    speed_profile = [lon_poly.calc_first_derivative(tt) for tt in t_steps ]

                    traj.d = [lat_poly.calc_point(tt) for tt in t_steps]
                    traj.x = []
                    traj.y = []

                    track_length = self.converter.reference_line.s[-1]
                    for s_val, d_val in zip(traj.s, traj.d):
                        wrapped_s = (s_val % track_length)
                        x, y = (self.converter.frenet_to_cartesian(wrapped_s, d_val))
                        traj.x.append(x)
                        traj.y.append(y)

                    lateral_jerk_cost = sum(lat_poly.calc_third_derivative(tt)**2 for tt in t_steps)
                    longitudinal_jerk_cost = sum(lon_poly.calc_third_derivative(tt)**2 for tt in t_steps )
                    velocity_cost = sum((v - target_v)**2 for v in speed_profile)
                    
                    offset_cost = abs( traj.d[-1] - target_d )

                    time_cost = 0.1 * T
                    base_cost = (0.1 * lateral_jerk_cost  + 0.05 * longitudinal_jerk_cost + 0.05 * velocity_cost + offset_cost + time_cost)
                    collision = False
                    # for i, t in enumerate(traj.t):
                    for i in range(0,len(traj.t),self.collision_step_skip):
                        t = traj.t[i]
                        for obs in self.obstacles:
                            obs_s, obs_d = obs.predict(t)
                            gap = obs_s - traj.s[i]
                            ttc = self.compute_ttc(gap, current_v, obs.v)
                            ttc_penalty = 0
                            if 0 < ttc < 3 :
                                ttc_penalty = max(ttc_penalty,500)
                            obs_s = (obs_s % track_length)
                            obs_x, obs_y = self.converter.frenet_to_cartesian(obs_s, obs_d)                            
                            distance = np.hypot(traj.x[i] - obs_x, traj.y[i] - obs_y)
                            if distance < self.safety_radius:                            
                                collision = True
                                break 
                        if collision:
                            break 
                    base_cost += ttc_penalty                        
                    if collision:
                        traj.cost = float("inf")
                        traj.valid = False
                    else:
                        traj.cost = base_cost
                        traj.valid = True
                    candidate_trajectories.append(traj)
        valid_paths = [traj for traj in candidate_trajectories if traj.cost != float("inf")]


        # print(
        #     "Collision paths:",
        #     len(candidate_trajectories)
        #     -
        #     len(valid_paths)
        #     )

        # print(
        #     "Valid paths:",
        #     len(valid_paths)
        #     )       
        if len(valid_paths) == 0:
            return None, candidate_trajectories
        best_path = min(valid_paths,key=lambda p: p.cost)
        return (best_path, candidate_trajectories)        