from dataclasses import dataclass
import numpy as np
@dataclass

class FrenetTrajectory :

    def __init__(self):

        self.s     =   []
        self.d     =   []
        self.x     =   []
        self.y     =   []
        self.cost  =   0.0
        self.valid = True