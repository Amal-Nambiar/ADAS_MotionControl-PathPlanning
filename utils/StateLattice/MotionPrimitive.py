import numpy as np
from utils.geometry import *


class MotionPrimitiveGenerator :

    def __init__(self, motion_model, heuristic):

        self.motion_model = motion_model
        self.heuristic    = heuristic

    def generate_library(self, max_steer):
        steers = np.linspace(-max_steer, max_steer, 7)
        library = []

        for steer in steers:
            primitive = []
            x, y, yaw = 0.0, 0.0, 0.0

            for _ in range(10):
                x, y, yaw = self.motion_model.propagate(x, y, yaw, steer, 1, step_size=1)
                primitive.append((x, y, yaw))
            
            # FIX: Append a dictionary containing both the path AND the steering meta-data
            library.append({
                "path": primitive,
                "steer": steer,
                "direction": 1
            })

        return library


    def transform_primitive(self, primitive, x, y, yaw):

        transformed = []

        R = np.array([
            [np.cos(yaw), -np.sin(yaw)],
            [np.sin(yaw), np.cos(yaw)]
        ])

        for (px, py, pyaw) in primitive :

            local_xy = np.array([px, py])
            global_xy = R @ local_xy

            gx = global_xy[0] + x
            gy = global_xy[1] + y

            gyaw = pyaw + yaw
            gyaw = wrap_to_pi(gyaw)

            transformed.append((gx,gy, gyaw))

        return transformed

    def prim_heuritic(self, node) :

        x = int(round(node.x))
        y = int(round(node.y))

        return self.hmap[y,x]

