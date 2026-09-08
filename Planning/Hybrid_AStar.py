import os
import sys
import heapq
import numpy as np
import copy
current_dir = os.path.dirname(os.path.abspath(__file__))
parent_dir  = os.path.abspath(os.path.join(current_dir, '..'))

if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.grid_map import *
from utils.node import *
from utils.Hybrid.Hybrid_Node import *
from utils.Hybrid.motion_model import *
from utils.Hybrid.collision_checker import *
from utils.geometry import *

class HybridAStar :

    def __init__(self,grid_map, motion_model,collision_checker,start_pose, goal_pose ,reeds_connector,heuristic_map, start_yaw = 0.0):

        self.grid_map            = grid_map
        self.motion_model        = motion_model
        self.start_pose          = start_pose
        self.goal_pose           = goal_pose
        self.start_yaw           = start_yaw
        self.open_set            = {}
        self.closed_set          = {}
        self.search_history      = []
        self.rs_connect_point    = []
        self.goal_node           = None  
        self.collision_checker   = collision_checker
        self.reeds_connector     = reeds_connector
        self.hmap                = heuristic_map

    def calc_index(self, node, yaw_res_deg = 15):
        x_idx                    = round(node.x)
        y_idx                    = round(node.y)
        yaw_idx                  = int(round(np.rad2deg(node.yaw))/yaw_res_deg)
        return x_idx, y_idx,yaw_idx


    def heuristic(self,node):
            x = int(round(node.x))
            y = int(round(node.y))
            return self.hmap[y,x]

    def reached_goal(self,node):
        dist = np.hypot(
            node.x - self.goal_pose[0],
            node.y - self.goal_pose[1]
        )    
        yaw_error = abs(wrap_to_pi(node.yaw - self.goal_pose[2]))
        return dist < 1.0 and yaw_error < np.deg2rad(15)

    def plan(self):

        start_node = HybridNode(
            x=self.start_pose[0],
            y=self.start_pose[1],
            yaw=self.start_yaw,
            cost=0.0
        )

        start_idx = self.calc_index(start_node)
        self.open_set[start_idx] = start_node
        heap = []
        heapq.heappush(heap,(self.heuristic(start_node), start_idx))

        while heap:
            _, current_idx = heapq.heappop(heap)
            if current_idx not in self.open_set:
                continue
            current = self.open_set[current_idx]
            del self.open_set[current_idx]
            self.closed_set[current_idx] = current

            if len(self.closed_set) % 6 == 0:
                self.search_history.append({
                    "open": copy.deepcopy(self.open_set),
                    "close": copy.deepcopy(self.closed_set)
                })
               
            dist_to_goal = np.hypot(current.x - self.goal_pose[0],current.y - self.goal_pose[1])    
            yaw_error = abs(wrap_to_pi(current.yaw - self.goal_pose[2]))
            if dist_to_goal < 4 and yaw_error < np.deg2rad(20):
                rs_path = (self.reeds_connector.connect_to_goal(current, (self.goal_pose[0],self.goal_pose[1],0.0) ))
                if rs_path is not None:
                    # print("Analytic Expansion Success", current.x, current.y)
                    # print("RS Path Points:", len(rs_path.x))
                    self.goal_node    = current
                    self.reeds_path   = rs_path
                    break

            if self.reached_goal(current):
                print("Goal Found")
                self.goal_node = current
                break

            children = (self.motion_model.generate_successors(current, np.deg2rad(35)))

            for child in children:
                child_idx = self.calc_index(child)
                if child_idx in self.closed_set:
                    continue
                if self.collision_checker.is_collision(child):
                    continue
                cost = current.cost + 1.0
                if child.direction == -1:
                    cost += 5.0

                cost += abs(np.rad2deg(child.steer)) * 0.05                
                cost += (abs(child.steer - current.steer)*20)

                child.cost = cost
                child.parent_index = current_idx
                priority = (child.cost + 5*self.heuristic(child))

                if (child_idx not in self.open_set):
                    self.open_set[child_idx] = child
                    heapq.heappush(heap,(priority,child_idx))  
                else:
                    if child.cost < self.open_set[child_idx].cost:
                        self.open_set[child_idx] = child
                        heapq.heappush(heap,(priority,child_idx))

        # print("Open:",len(self.open_set))        
        # print("Closed:",len(self.closed_set))

    def reconstruct_path(self):
        rx = []
        ry = []
        current = self.goal_node
        while current is not None:
            rx.append(current.x)
            ry.append(current.y)
            if current.parent_index is None:
                break
            current = self.closed_set.get(current.parent_index)
        rx.reverse()
        ry.reverse()
        if hasattr(self,"reeds_path"):
            rx.extend(
                list(self.reeds_path.x)
            )
            ry.extend(list(self.reeds_path.y))
        return rx, ry    

    