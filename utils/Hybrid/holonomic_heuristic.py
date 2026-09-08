import numpy as np
import heapq


import numpy as np
import heapq


class HolonomicHeuristic:

    def __init__(self, grid_map):

        self.grid_map = grid_map

        self.motion = [
            (1,0,1.0),
            (0,1,1.0),
            (-1,0,1.0),
            (0,-1,1.0),

            (1,1,np.sqrt(2)),
            (-1,1,np.sqrt(2)),
            (1,-1,np.sqrt(2)),
            (-1,-1,np.sqrt(2))
        ]

    def build(self, goal_pose):

        width = self.grid_map.width
        height = self.grid_map.height

        hmap = np.full((height, width),np.inf)
        gx = int(goal_pose[0])
        gy = int(goal_pose[1])
        heap = []
        heapq.heappush(heap, (0.0, gx,gy))
        count = 0       

        while heap:
            cost, x,y = heapq.heappop(heap)
            count += 1
            # if count % 1000 == 0:
            #     print(count, x,y)
            

            for dx, dy , move_cost in self.motion:

                nx = x + dx
                ny = y + dy

                if not (0 <= nx < width and 0 <= ny < height):
                    continue

                if self.grid_map.is_occupied(nx,ny,inflation_radius=2):
                    continue
                new_cost = cost + move_cost
                if new_cost < hmap[ny,nx]:
                    hmap[ny, nx] = new_cost
                    heapq.heappush(heap, (new_cost, nx, ny))
        return hmap


