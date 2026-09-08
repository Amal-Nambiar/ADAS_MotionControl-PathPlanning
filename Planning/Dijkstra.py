import numpy as np
import math
import os
import sys
import matplotlib.pyplot as plt
import heapq  # Added for O(log N) sorting priority queue processing
import copy

current_dir = os.getcwd()
parent_dir  = os.path.abspath(os.path.join(current_dir, ".."))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.grid_map import *
from utils.Different_map import *
from utils.node import Node

class Dijkstra:

    def __init__(self, grid_map, start, goal):
        self.grid_map         = grid_map
        self.start            = start
        self.goal             = goal
        self.open_set         = {}
        self.close_set        = {}      
        self.heap             = []  # The binary heap array containing elements ordered as: (cost, unique_idx)
        self.search_history   = []
        self.motion           = [
            (1,0,1.0),(0,1,1.0),(-1,0,1.0),(0,-1,1.0),        
            (1,1,np.sqrt(2)),(-1,1,np.sqrt(2)),(1,-1,np.sqrt(2)),(-1,-1,np.sqrt(2))
            ] 

    def calc_index(self, x, y):
        return y * self.grid_map.width + x    

    def plan(self):
        start_node = Node(x=self.start[0], y=self.start[1], cost=0.0, parent_index=-1)
        start_idx  = self.calc_index(start_node.x, start_node.y)      
        self.open_set[start_idx] = start_node
        heapq.heappush(self.heap, (start_node.cost, start_idx))        
        self.goal_node = None

        while self.heap:
            cost, current_id = heapq.heappop(self.heap)

            if current_id in self.close_set:
                continue
                
            current = self.open_set[current_id]

            if len(self.close_set) % 6 == 0:
                self.search_history.append({
                    "open": copy.deepcopy(self.open_set),
                    "close": copy.deepcopy(self.close_set)
                })

            if current.x == self.goal[0] and current.y == self.goal[1]:
                print("Goal Found")
                self.goal_node = current
                break
                
            del self.open_set[current_id]
            self.close_set[current_id] = current

            for dx, dy, move_cost in self.motion:
                nx = current.x + dx
                ny = current.y + dy
                
                if self.grid_map.is_occupied(nx, ny,inflation_radius = 2):
                    continue
                    
                new_cost = current.cost + move_cost
                neighbour_idx = self.calc_index(nx, ny)
                
                if neighbour_idx in self.close_set:
                    continue
                    
                # Scenario A: Brand new unvisited grid cell node discovered
                if neighbour_idx not in self.open_set:
                    neighbour = Node(x=nx, y=ny, cost=new_cost, parent_index=current_id)
                    self.open_set[neighbour_idx] = neighbour
                    heapq.heappush(self.heap, (new_cost, neighbour_idx))
                    
                # Scenario B: Node exists in frontier, check if new path is a shorter shortcut
                else:
                    if self.open_set[neighbour_idx].cost > new_cost:
                        self.open_set[neighbour_idx].cost = new_cost
                        self.open_set[neighbour_idx].parent_index = current_id
                        heapq.heappush(self.heap, (new_cost, neighbour_idx))

    def reconstruct_path(self, goal_node):
        if goal_node is None:
            return [], []
        rx, ry = [], []
        current = goal_node
        while current.parent_index != -1:
            rx.append(current.x)
            ry.append(current.y)
            current = self.close_set[current.parent_index]

        rx.append(self.start[0])
        ry.append(self.start[1])
        return rx[::-1], ry[::-1]
