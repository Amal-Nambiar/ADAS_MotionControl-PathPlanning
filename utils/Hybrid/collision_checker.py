import os
import sys
import numpy as np

current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))

if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from Vehicle.vehicle_geometry import get_vehicle_polygon

class CollisionChecker:

    def __init__(self, grid_map, Vehicle_cfg):
        self.cfg                 = Vehicle_cfg
        self.grid_map            = grid_map

    def is_collision(self, node):

        polygon = get_vehicle_polygon(node.x,node.y,node.yaw,self.cfg)
        polygon = np.vstack((polygon, polygon[0]))

        for i in range(len(polygon)-1):
            p1 = polygon[i]
            p2 = polygon[i+1]
            distance = np.linalg.norm(p2 - p1)

            n_samples = max(2,int(distance * 2))

            for alpha in np.linspace(0,1,n_samples):
                point = (p1 + alpha*(p2-p1))
                gx = int(round(point[0]))
                gy = int(round(point[1]))

                if self.grid_map.is_occupied(gx,gy,inflation_radius = 1):
                    return True
        return False
  



    