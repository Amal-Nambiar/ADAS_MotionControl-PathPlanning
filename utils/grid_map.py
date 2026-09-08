import numpy as np
import matplotlib.pyplot as plt
class GridMap:

    def __init__(self, width =80 , height = 50, resolution = 200):
        self.width               =  width
        self.height              =  height
        self.resolution          =  resolution
        self.grid                =  np.zeros((height, width), dtype = np.uint8)

    def set_obstacle(self, x , y):
        self.grid[y,x]           =  1

    def is_inside(self, x, y ):
        return (0 <= x < self.width and 0 <= y < self.height)

    def is_occupied(self, x, y, inflation_radius=3.0):
        for dx in range(-inflation_radius, inflation_radius + 1):
            for dy in range(-inflation_radius, inflation_radius + 1):
                nx, ny = x + dx, y + dy
                if nx < 0 or nx >= self.width or ny < 0 or ny >= self.height:
                    return True
                if self.grid[ny, nx] == 1: 
                    return True
        return False


    def plot_vertical_obstacle(self, x_pos, y_start, y_end):
        for y in range(y_start,y_end):
            self.set_obstacle(x_pos,y)

    def plot_horizontal_obstacle(self, y_pos, x_start, x_end):
        for x in range(x_start,x_end):
            self.set_obstacle(x, y_pos)

    def border(self):        
        for i in range(self.height):
            self.set_obstacle(0,i)
        for i in range(self.height):
            self.set_obstacle(self.width - 1, i)
        for i in range(self.width):
            self.set_obstacle(i,0)
        for i in range(self.width):
            self.set_obstacle(i, self.height - 1)

    def plot_block_obstacle(self, x_start, y_start, width, height):
        for x in range(x_start, x_start + width):
            for y in range(y_start, y_start + height):
                self.set_obstacle(x, y)

    def plot_circular_obstacle(self, x_center, y_center, radius):
        x_start = max(0, x_center - radius)
        x_end   = min(self.width, x_center + radius + 1)
        y_start = max(0, y_center - radius)
        y_end   = min(self.height, y_center + radius + 1)
        for x in range(x_start, x_end):
            for y in range(y_start, y_end):
                if (x - x_center) ** 2 + (y - y_center) ** 2 <= radius ** 2:
                    self.set_obstacle(x, y)

    def plot_grid(self):
        plt.figure(figsize=(8,5))
        plt.imshow(self.grid,origin = "lower", cmap = "Greys")
        plt.grid(True)
        plt.show

    





