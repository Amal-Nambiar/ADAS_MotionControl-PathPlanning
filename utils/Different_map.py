import sys
import os

current_dir = os.getcwd()
parent_dir  = os.path.abspath(os.path.join(current_dir,".."))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from utils.grid_map import GridMap

class Map:

    def __init__(self):
        pass

    def map_1_simple(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_vertical_obstacle(x_pos=30, y_start=10, y_end=40)
        self.grid_map.plot_horizontal_obstacle(y_pos=25, x_start=40, x_end=70)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_2_simple(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_vertical_obstacle(x_pos=10, y_start=10, y_end=30)
        self.grid_map.plot_vertical_obstacle(x_pos=35, y_start=20, y_end=40)
        self.grid_map.plot_vertical_obstacle(x_pos=60, y_start=10, y_end=40)
        # self.grid_map.plot_grid()        
        return self.grid_map

    def map_3_simple(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_vertical_obstacle(x_pos=10, y_start=10, y_end=30)
        self.grid_map.plot_vertical_obstacle(x_pos=35, y_start=20, y_end=40)
        self.grid_map.plot_vertical_obstacle(x_pos=60, y_start=10, y_end=40)
        self.grid_map.plot_horizontal_obstacle(y_pos=40, x_start=35, x_end=60)
        # self.grid_map.plot_grid()           
        return self.grid_map

    def map_4_simple(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()        
        self.grid_map.plot_vertical_obstacle(x_pos=30, y_start=10, y_end=40)
        self.grid_map.plot_horizontal_obstacle(y_pos=40, x_start=10, x_end=30)
        self.grid_map.plot_horizontal_obstacle(y_pos=10, x_start=30, x_end=65)  
        # self.grid_map.plot_grid()  
        return self.grid_map
    
    def map_5_maze(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        x_step = width // 8
        for x in range(x_step, width - x_step, x_step):
            y_end = int(height * 0.8) if x % (2 * x_step) == 0 else int(0.6*height)
            self.grid_map.plot_vertical_obstacle(x_pos=x, y_start=height//6, y_end=y_end)

        self.grid_map.plot_block_obstacle(
            x_start=int(width * 0.18), y_start=int(height * 0.3), 
            width=int(width * 0.18), height=int(height * 0.1)
        )
        self.grid_map.plot_block_obstacle(
            x_start=int(width * 0.56), y_start=int(height * 0.6), 
            width=int(width * 0.18), height=int(height * 0.1)
        )        
        # self.grid_map.plot_grid()
        return self.grid_map


    def map_6_gated_wall(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_block_obstacle(x_start=38, y_start=1, width=4, height=30)
        self.grid_map.plot_block_obstacle(x_start=38, y_start=40, width=4, height=30)
        self.grid_map.plot_block_obstacle(x_start=15, y_start=15, width=10, height=20)
        self.grid_map.plot_block_obstacle(x_start=55, y_start=15, width=10, height=50)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_7_narrow_chokepoints(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_block_obstacle(x_start=15, y_start=15, width=12, height=34)
        self.grid_map.plot_block_obstacle(x_start=45, y_start=15, width=12, height=34)
        self.grid_map.plot_block_obstacle(x_start=30, y_start=1, width=12, height=34)
        self.grid_map.plot_block_obstacle(x_start=60, y_start=1, width=12, height=34)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_8_the_spiral_trap(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_horizontal_obstacle(y_pos=8, x_start=8, x_end=72)
        self.grid_map.plot_vertical_obstacle(x_pos=72, y_start=8, y_end=42)
        self.grid_map.plot_horizontal_obstacle(y_pos=42, x_start=16, x_end=72)
        self.grid_map.plot_vertical_obstacle(x_pos=16, y_start=16, y_end=42)
        self.grid_map.plot_horizontal_obstacle(y_pos=16, x_start=16, x_end=64)
        self.grid_map.plot_vertical_obstacle(x_pos=64, y_start=16, y_end=34)
        self.grid_map.plot_horizontal_obstacle(y_pos=34, x_start=24, x_end=64)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_9_scattered_forest(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        for x in range(int(width*0.1), int(width * 0.9), 8):
            for y in range(int(height*0.15), int(height*0.95), 10):
                w, h = (4, 2) if (x + y) % 2 == 0 else (2, 4)
                self.grid_map.plot_block_obstacle(x_start=x, y_start=y, width=w, height=h)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_10_central_fortress(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_block_obstacle(x_start=int(width * 0.05), y_start=10, width=5, height=30)
        self.grid_map.plot_block_obstacle(x_start=int(width * 0.8), y_start=10, width=5, height=30)
        self.grid_map.plot_horizontal_obstacle(y_pos=int(height*0.3), x_start=25, x_end=55) 
        self.grid_map.plot_horizontal_obstacle(y_pos=int(height*0.7), x_start=25, x_end=55)
        self.grid_map.plot_vertical_obstacle(x_pos=int(width*0.2), y_start=15, y_end=35)   
        self.grid_map.plot_vertical_obstacle(x_pos=int(width*0.7), y_start=20, y_end=35)   
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_11_bubble_field(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.3), y_center=25, radius=8)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.7), y_center=25, radius=8)   
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.5), y_center=10, radius=4)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.5), y_center=40, radius=4)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.125), y_center=40, radius=3)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.875), y_center=10, radius=3)     
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_12_industrial_warehouse(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_block_obstacle(x_start=int(width*0.1875), y_start=10, width=4, height=30)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.4375), y_start=10, width=4, height=30)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.6875), y_start=10, width=4, height=30)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.3125), y_center=15, radius=3)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.3125), y_center=35, radius=3)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.5625), y_center=15, radius=3)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.5625), y_center=35, radius=3)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_13_slalom_run(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_block_obstacle(x_start=int(width*0.0125), y_start=20, width=20, height=10)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.75), y_start=20, width=19, height=10)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.3125), y_center=38, radius=5)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.35), y_center=12, radius=6)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.6875), y_center=38, radius=5)
        # self.grid_map.plot_grid()
        return self.grid_map

    def map_14_the_pinball_wizard(self, width=80, height=50):
        self.grid_map = GridMap(width=width, height=height)
        self.grid_map.border()
        self.grid_map.plot_block_obstacle(x_start=int(width*0.45), y_start=20, width=8, height=10)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.25), y_center=25, radius=6)
        self.grid_map.plot_circular_obstacle(x_center=int(width*0.75), y_center=25, radius=6)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.125), y_start=6, width=10, height=4)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.75), y_start=6, width=10, height=4)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.125), y_start=40, width=10, height=4)
        self.grid_map.plot_block_obstacle(x_start=int(width*0.75), y_start=40, width=10, height=4)
        # self.grid_map.plot_grid()
        return self.grid_map
