import matplotlib.pyplot as plt
import numpy as np
from vis_loc.camera import Camera
from vis_loc.rendering import Render
from vis_loc.utilities import wrap_angle

class Robot:
    def __init__(self, x, y, heading, ax, field, debug:bool) -> None:
        self.location:list = [x, y, heading]
        self.ax = ax
        self.field = field
        self.render = Render(ax)
        self.camera = Camera(range=12, fov=120, position=self.location, field=field, ax=ax, debug=debug)

        # visible robot components
        self._marker = None
        self._arrow = None
        

    def draw_robot(self): # TODO: move to the render class after moving camera.raycast
        '''
        draws/renders the robot (a square with an arrow) on the field plot.
        '''
        
        # remove previous artists 
        self.render.remove_element(self._marker)
        self.render.remove_element(self._arrow)

        x, y, heading = self.location
        # marker (note plotting uses y,x to match existing code)
        self._marker, = self.ax.plot(y, x, 'sc', markersize=20)

        dx = 0.4 * np.cos(np.radians(heading)) # rotate heading to match weird 
        dy = 0.4 * np.sin(np.radians(heading))
        self._arrow = self.ax.arrow(y, x, dy, dx, head_width=0.3, head_length=0.3,
                                    fc='red', ec='red', linewidth=2)

        # draw camera rays and update
        self.camera.raycast()
        self.render.render()


    def drive_path(self, dx=0.0, dy=0.0, dth=0.0, steps=1, animate=True):
        '''
        moves the robot to a new spot. possibly redundant function.
        will soon be replaced by seaprate drive, turn, pose, and path functions.

        Args:
            dx: delta x
            dy: delta y
            dth: delta heading (degrees)
            steps: number of steps to move over
            animate: default is True. animates the robot as it moves

        Returns:
            self.location: the updated location of the robot
        '''

        x0, y0, th0 = self.location
        if steps < 1:
            steps = 1

        xs = np.linspace(x0, x0 + dx, steps + 1)[1:]
        ys = np.linspace(y0, y0 + dy, steps + 1)[1:]
        ths = np.linspace(th0, th0 + dth, steps + 1)[1:]

        for xi, yi, thi in zip(xs, ys, ths):
            th_wrapped = wrap_angle(thi)
            self.location = [xi, yi, th_wrapped]
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

