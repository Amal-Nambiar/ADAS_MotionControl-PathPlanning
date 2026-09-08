from dataclasses import dataclass
@dataclass

class FrenetSnapshot :

    vehicle_state          : object

    obstacles_x            : list
    obstacles_y            : list

    best_x                 : list
    best_y                 : list
    
    candidate_paths        : list

