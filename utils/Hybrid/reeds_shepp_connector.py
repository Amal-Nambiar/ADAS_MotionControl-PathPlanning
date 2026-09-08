import numpy as np
from . import reeds_shepp
from .Hybrid_Node import HybridNode

class ReedsSheppConnector :

    def __init__(self, collision_checker,vehicle_cfg):

        self.vehicle_cfg = vehicle_cfg
        self.collision_checker = collision_checker

    def connect_to_goal(self, current, goal_pose):
        turning_radius  = (self.vehicle_cfg.wheelbase / np.tan(self.vehicle_cfg.max_steer))
        max_curvature   = 1.0/turning_radius
        path = reeds_shepp.calc_optimal_path(sx= current.x, sy = current.y, syaw = current.yaw, 
                                             gx = goal_pose[0], gy = goal_pose[1], gyaw = goal_pose[2], maxc = max_curvature)

        for x, y , yaw in zip(path.x, path.y, path.yaw):

            node = HybridNode(x = x, y= y, yaw = yaw)
            if self.collision_checker.is_collision(node):
                return None

        return path

    