from dataclasses import dataclass
@dataclass

class HybridNode :

    x                :  float
    y                :  float
    yaw              :  float
    cost             :  float = 0.0
    parent_index     :  int = -1
    direction        :  int = 1
    steer            :  float = 0.0
     