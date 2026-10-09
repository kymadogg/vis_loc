import numpy as np
import matplotlib.pyplot as plt
from utilities import liang_barsky

class Camera:
    def __init__(self, range, fov, position, field, ax, debug:bool) -> None:
        self.range = range
        self.fov = fov
        self.position = position
        self.field = field  # field reference
        self.goal_coords = field.goal_coords
        self.goal_size = field.goal_size
        self.ax = ax
        self.rays = []
        self.debug = debug

    def raycast(self):
        x, y, heading = self.position

        # remove previously drawn rays
        for ray in self.rays:
            try:
                ray.remove()
            except Exception:
                pass
        self.rays = []

        start_angle = heading - self.fov / 2
        end_angle = heading + self.fov / 2

        for angle in np.arange(start_angle, end_angle + 1, 1):
            rad = np.radians(angle)

            x_end = x + self.range * np.cos(rad)
            y_end = y + self.range * np.sin(rad)

            # self.ax.plot([y, y_end], [x, x_end], 'b-', alpha=0.3, linewidth=0.5)
            # self.rays.append(((x, y), (x_end, y_end)))

            line, = self.ax.plot([y, y_end], [x, x_end], 'b-', alpha=0.3, linewidth=0.5)
            self.rays.append(line)

    def show_output(self):
        fig, ax = plt.subplots(figsize=(14, 3))

        readings = self.read()

        # If nothing is visible, show an empty view and return.
        if not readings:
            ax.set_title('Camera Output (no visible goals)')
            ax.set_xlabel('Relative Degree (0 = straight ahead)')
            ax.axvline(x=0, color='black', linestyle='--', alpha=0.5)
            plt.show()
            plt.close(fig)
            return

        colors = []
        xs = []

        for color, rel in readings:
            xs.append(rel)
            if color == 0:
                colors.append('black')
            elif color == 1:
                colors.append('blue')
            elif color == 2:
                colors.append('red')
            else:
                colors.append('gray')

        ax.bar(xs, np.ones(len(xs)), color=colors, width=0.8)
        ax.set_xlabel('Relative Degree (0 = straight ahead)')
        ax.set_title('Camera Output (occlusion-aware)')
        ax.axvline(x=0, color='black', linestyle='--', alpha=0.5)
        plt.show()
        plt.close(fig)

    def read(self):
        '''read the camera sensor and return visible goals'''
        
        px, py, heading = self.position
        half_size = self.goal_size / 2
        half_fov = self.fov / 2
        readings = []

        for i, (gx, gy) in enumerate(self.goal_coords):
            vx = gx - px
            vy = gy - py
            dist = np.hypot(vx, vy)
            abs_heading = np.degrees(np.arctan2(vy, vx))
            rel = ((abs_heading - heading + 180) % 360) - 180

            visible = True
            if dist > self.range or abs(rel) > half_fov:
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
                color = 0 # center
            elif 1 <= i <= 4:
                color = 3
            elif 5 <= i <= 6:
                color = 1
            else:
                color = 2 # blue?

            readings.append((color, rel if visible else None))

        visible_readings = [r for r in readings if r[1] is not None]
        
        if self.debug:
            print(visible_readings)

        return visible_readings
