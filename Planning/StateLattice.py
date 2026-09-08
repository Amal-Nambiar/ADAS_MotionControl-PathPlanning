import heapq
import numpy as np
import copy
from utils.StateLattice.node import *
from utils.geometry import *

class StateLatticePlanner:

    def __init__(self, grid_map, primitive_library, collision_checker, MotionPrimitive,reeds_connector=None):
        self.grid_map               = grid_map
        self.primitive_library      = primitive_library  
        self.collision_checker      = collision_checker
        self.MotionPrimitive        = MotionPrimitive
        self.goal_node              = None
        self.reeds_connector        = reeds_connector
        self.search_history         = []
        self.open_set               = {}
        self.closed_set             = {}

    def reached_goal(self, node, goal_x, goal_y):
        dist = np.hypot(node.x - goal_x, node.y - goal_y)
        return dist < 2 

    def heuristic(self, node):
        x = int(round(node.x))
        y = int(round(node.y))
        
        if 0 <= y < self.hmap.shape[0] and 0 <= x < self.hmap.shape[1]:
            val = self.hmap[y, x]
            if val == 0 or val == 1:
                return np.hypot(node.x - self.goal_pose[0], node.y - self.goal_pose[1])
            return val
        return np.hypot(node.x - self.goal_pose[0], node.y - self.goal_pose[1])

    def cal_index(self, node, yaw_res_deg=15):
        x_idx = int(np.floor(node.x / 2.0))
        y_idx = int(np.floor(node.y / 2.0))
        yaw_idx = wrap_to_pi(node.yaw)
        yaw_idx = int(np.floor(yaw_idx / np.deg2rad(yaw_res_deg)))
        return (x_idx, y_idx, yaw_idx)

    def apply_primitive(self, prim_info, node):
        primitive = prim_info["path"]
        transformed = self.MotionPrimitive.transform_primitive(primitive, node.x, node.y, node.yaw)
        
        end_x       = transformed[-1][0]
        end_y       = transformed[-1][1]
        end_yaw     = transformed[-1][2]
        
        child       = LatticeNode(x=end_x, y=end_y, yaw=end_yaw)
        child.direction = prim_info["direction"]
        child.steer     = prim_info["steer"]
        child.path      = transformed 

        for (x, y, yaw) in transformed:
            test_node = LatticeNode(x=x, y=y, yaw=yaw)
            if self.collision_checker.is_collision(test_node):
                return None        
        return child

    def plan(self, start_pose, goal_pose, hmap):
        self.start_pose = start_pose
        self.start_yaw  = start_pose[2]
        self.goal_pose  = goal_pose
        self.hmap       = hmap
        self.open_set.clear()
        self.closed_set.clear()
        self.search_history.clear()  
        self.goal_node = None
    
        start_node = LatticeNode(
            x=self.start_pose[0], 
            y=self.start_pose[1], 
            yaw=self.start_yaw, 
            cost=0.0
        )
        start_node.steer = 0.0
        start_node.direction = 1
        start_node.id = 0
        start_node.parent = None  
        node_id_counter = 1

        start_idx = self.cal_index(start_node)
        self.open_set[start_idx] = start_node
        heap = []
        heapq.heappush(heap, (self.heuristic(start_node), start_idx))

        iterations = 0
        while heap:
            iterations += 1
            _, current_idx = heapq.heappop(heap)
            if current_idx not in self.open_set:
                continue
            current = self.open_set[current_idx]
            del self.open_set[current_idx]
            
            if current_idx in self.closed_set:
                continue
            self.closed_set[current_idx] = current

            if len(self.closed_set) % 1 == 0:
                current_primitives = []
                for node in self.closed_set.values():
                    if hasattr(node, 'path') and node.path:
                        current_primitives.append(node.path)

                self.search_history.append({
                    "open": copy.deepcopy(self.open_set),
                    "close": copy.deepcopy(self.closed_set),
                    "primitives": current_primitives
                })

            dist_to_goal = np.hypot(current.x - self.goal_pose[0], current.y - self.goal_pose[1])    
            yaw_error = abs(wrap_to_pi(current.yaw - self.goal_pose[2]))

            if self.reached_goal(current, self.goal_pose[0], self.goal_pose[1]) and yaw_error < np.deg2rad(45):
                print(f'Goal reached successfully! after {iterations} iterations')
                self.goal_node = current
                return True

            for prim_info in self.primitive_library:
                child = self.apply_primitive(prim_info, current)
                if child is None:
                    continue
                child_idx = self.cal_index(child)
                if child_idx in self.closed_set:
                    continue
                
                cost = current.cost + 10.0
                cost += abs(np.rad2deg(child.steer)) * 0.1                
                cost += (abs(child.steer - current.steer) * 30)

                child.cost = cost
                child.parent = current 
                child.id = node_id_counter
                node_id_counter += 1
                
                priority = (child.cost + 3.0 * self.heuristic(child))

                if child_idx not in self.open_set or child.cost < self.open_set[child_idx].cost:
                    self.open_set[child_idx] = child
                    heapq.heappush(heap, (priority, child_idx))

        print("Failed to find a path.")
        return False


    def reconstruct_path(self):
        if self.goal_node is None:
            return [], []
        rx = []
        ry = []
        current = self.goal_node
        
        while current is not None and hasattr(current, 'path') and current.path:
            segment_x = [p[0] for p in current.path]
            segment_y = [p[1] for p in current.path]
            
            rx = segment_x + rx
            ry = segment_y + ry
            
            current = current.parent 
            
        return rx, ry    
