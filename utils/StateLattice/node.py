from dataclasses import dataclass
@dataclass

class LatticeNode:

    x               : float
    y               : float
    yaw             : float    
    cost            : float = 0.0
    parent_index    : tuple = None