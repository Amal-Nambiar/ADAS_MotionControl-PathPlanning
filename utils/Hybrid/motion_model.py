import os
import sys
import numpy as np
from .Hybrid_Node import HybridNode

current_dir = os.getcwd()
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))

if parent_dir not in sys.path:
    sys.path.append(parent_dir)
from utils.geometry import *

class MotionModel :

    def __init__(self, wheelbase):
        self.wheelbase  = wheelbase

    def propagate(self,x,y,yaw,steer,direction,step_size):
        ds = 0.2
        n_steps = int(step_size/ds)
        for _ in range(n_steps):
            v        =       direction * ds
            x        +=      v * np.cos(yaw)
            y        +=      v * np.sin(yaw)
            yaw      +=      (v/self.wheelbase) * np.tan(steer)
            yaw      =       wrap_to_pi(yaw)
        return x,y,yaw

    def generate_successors(self, node, max_steer):
        # steers =  [ -max_steer, 0.0 , max_steer]
        steers = np.linspace(-max_steer, max_steer, 20)
        directions = [1, -1]
        successors =[]
        for direction in directions : 
            for steer in steers:
                nx, ny, nyaw = self.propagate(node.x, node.y, node.yaw, steer, direction,step_size=2.0)
                child        = HybridNode(x = nx, y =ny, yaw = nyaw, direction = direction)
                successors.append(child)
        return successors
    

    def calc_hybrid_index(self,x,y,yaw,yaw_resolution_deg = 15):
        x_idx = round(x)
        y_idx = round(y)
        yaw_idx = int(round(np.rad2deg(yaw)/yaw_resolution_deg))
        return x_idx, y_idx, yaw_idx
    
    