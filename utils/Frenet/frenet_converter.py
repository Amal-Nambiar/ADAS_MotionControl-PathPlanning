import numpy as np

class FrenetConverter:

    def __init__(self, reference_line):

        self.reference_line = reference_line

    def find_nearest_index(self, x ,y):
        dx = (self.reference_line.x - x)
        dy = (self.reference_line.y - y)
        distance = np.hypot(dx, dy)

        return np.argmin(distance)

    def cartesian_to_s(self,x,y):

        idx = self.find_nearest_index(x,y)
        return  (self.reference_line.s[idx])

    def cartesian_to_sd(self, x, y):
        idx = self.find_nearest_index(x,y)
        px = self.reference_line.x[idx]
        py = self.reference_line.y[idx]
        path_yaw = self.reference_line.yaw[idx]

        dx = x - px
        dy = y - py

        nx  = -np.sin(path_yaw)
        ny  = np.cos(path_yaw)
        d   = (dx * nx + dy * ny)
        s   = self.reference_line.s[idx]

        return s, d


    def frenet_to_cartesian(self, s, d):

        idx      = np.argmin(np.abs(np.array(self.reference_line.s) - s))
        px       = self.reference_line.x[idx]
        py       = self.reference_line.y[idx]
        path_yaw = self.reference_line.yaw[idx]

        nx       = -np.sin(path_yaw)
        ny       =  np.cos(path_yaw)
        x        =  px + d  * nx
        y        =  py + d  * ny
        return x,y


