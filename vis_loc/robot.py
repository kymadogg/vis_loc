import matplotlib.pyplot as plt
import numpy as np
from vis_loc.camera import Camera
from .utilities import wrap_angle

class Robot:
    def __init__(self, x, y, heading, ax, field, debug:bool) -> None:
        self.location = (x, y, heading)
        self.ax = ax
        self.field = field
        self.camera = Camera(range=12, fov=120, position=self.location, field=field, ax=ax, debug=debug)
        self._marker = None
        self._arrow = None

    def draw_robot(self):
        # remove previous artists
        if self._marker is not None:
            try:
                self._marker.remove()
            except Exception:
                pass
            self._marker = None
        if self._arrow is not None:
            try:
                self._arrow.remove()
            except Exception:
                pass
            self._arrow = None

        x, y, heading = self.location
        # marker (note plotting uses y,x to match existing code)
        self._marker, = self.ax.plot(y, x, 'sc', markersize=20)

        dx = 0.4 * np.cos(np.radians(heading))
        dy = 0.4 * np.sin(np.radians(heading))
        self._arrow = self.ax.arrow(y, x, dy, dx, head_width=0.3, head_length=0.3,
                                    fc='red', ec='red', linewidth=2)
        
        self.camera.raycast()

        try:
            self.ax.figure.canvas.draw_idle()
        except Exception:
            pass

    def drive_path(self, dx=0.0, dy=0.0, dth=0.0, steps=1, animate=True):
        """
        Shift robot by (dx, dy, dth) relative to current pose.
        - dx, dy: relative translation in x,y (same units as self.location)
        - dth: relative heading change in degrees
        - steps: interpolation steps (>=1)
        - animate: if True, pause briefly each step
        Returns the updated (x, y, th).
        """

        x0, y0, th0 = self.location
        if steps < 1:
            steps = 1

        xs = np.linspace(x0, x0 + dx, steps + 1)[1:]
        ys = np.linspace(y0, y0 + dy, steps + 1)[1:]
        ths = np.linspace(th0, th0 + dth, steps + 1)[1:]

        for xi, yi, thi in zip(xs, ys, ths):
            th_wrapped = wrap_angle(thi)
            self.location = (xi, yi, th_wrapped)
            try:
                self.camera.position = self.location
            except Exception:
                pass
            self.draw_robot()
            if animate:
                try:
                    self.ax.figure.canvas.draw_idle()
                    plt.pause(0.02)
                except Exception:
                    pass

        return self.location

