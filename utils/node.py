from dataclasses import dataclass

@dataclass

class Node:

    x                : int
    y                : int
    cost             : float = 0.0
    parent_index     : int   = -1.0
    