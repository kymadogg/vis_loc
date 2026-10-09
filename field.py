import matplotlib.pyplot as plt
import numpy as np
import math
from matplotlib.patches import Rectangle
from utilities import liang_barsky

class Field:
    def __init__(self) -> None:
        self.goal_size = 5/12 # in ft

        self.goal_coords = [ (0,0),   # center
            (-2, -4), (-4, -2), 
            (2, 4), (4,2), # neutral
            (2, -4), (4, -2), # red goals
            (-2, 4), (-4, 2) # blue goals
        ]
        # index 0 = center, 1-4 = neutral, 5-6 = red, 7-8 blue
    
    def plot_base(self):

        fig, ax = plt.subplots(figsize=(8, 8))

        ax.set_xlim(6, -6)
        ax.set_ylim(-6, 6)
        ax.set_aspect('equal', adjustable='box')

        ax.axhline(y=0, color='gray', linewidth=1, alpha=0.3)
        ax.axvline(x=0, color='gray', linewidth=1, alpha=0.3)

        # Draw goals as rectangles at actual size
        half_size = self.goal_size / 2
    
        colors_by_index = {
            0: 'black',      # center
            1: 'gray', 2: 'gray', 3: 'gray', 4: 'gray',  # neutral
            5: 'red', 6: 'red',      # red goals
            7: 'blue', 8: 'blue'     # blue goals
        }
        
        for index, (gx, gy) in enumerate(self.goal_coords):
            color = colors_by_index[index]
           # center is at (gx, gy)
            rect = Rectangle((gx - half_size, gy - half_size), 
                            self.goal_size, self.goal_size, 
                            color=color, alpha=0.7, ec='black', linewidth=1)
            ax.add_patch(rect)
    
        

        ax.set_box_aspect(1)
        ax.set_xlabel('Y (ft)')
        ax.set_ylabel('X (ft)')
        ax.set_title('VEX V5RC Override')
        ax.grid(True, alpha=0.2)

        return fig, ax

    def predicted_output(self, input_pos, fov:int, range:float):
        '''read the camera sensor and return visible goals'''
        
        px, py, heading = input_pos
        half_size = self.goal_size / 2
        half_fov = fov / 2
        readings = []

        for i, (gx, gy) in enumerate(self.goal_coords):
            vx = gx - px
            vy = gy - py
            dist = np.hypot(vx, vy)
            abs_heading = np.degrees(np.arctan2(vy, vx))
            rel = ((abs_heading - heading + 180) % 360) - 180

            visible = True
            if dist > range or abs(rel) > half_fov:
                visible = False
            else:
                # check occlusion by any other goal-rectangle
                for j, (ox, oy) in enumerate(self.goal_coords):
                    if j == i:
                        continue
                    hit, info = liang_barsky(px, py, gx, gy, ox, oy, half_size)
                    if hit and info is not None:
                        _, _, t_hit = info
                        # if intersection occurs before reaching target goal -> blocked
                        if t_hit * dist < dist - 1e-6:
                            visible = False
                            break

            # map index to color code (same as Field)
            if i == 0:
                color = 0
            elif 1 <= i <= 4:
                color = 3
            elif 5 <= i <= 6:
                color = 1
            else:
                color = 2

            readings.append((color, rel if visible else None))

        visible_readings = [r for r in readings if r[1] is not None]
        
        return visible_readings





