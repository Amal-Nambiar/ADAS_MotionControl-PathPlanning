import numpy as np

class reference_line:

    def __init__(self, x, y):

        self.x                   = x
        self.y                   = y
        self.s                   = []
        self.yaw                 = []
        self.calculate_s()

    def calculate_s(self):

        self.s = [0.0]

        for i in range(1 , len(self.x)):
            dx = self.x[i] - self.x[i-1]
            dy = self.y[i] - self.y[i-1]
            yaw = np.arctan2(dy,dx)
            self.yaw.append(yaw)
            ds = np.hypot(self.x[i] - self.x[i-1], self.y[i] - self.y[i-1])
            self.s.append(self.s[-1] + ds)
        if len(self.yaw) > 0:
            self.yaw.append(self.yaw[-1])
