import numpy as np
import math
from mpl_toolkits.axes_grid1 import make_axes_locatable
from utilities import wrap_angle, ang_diff
from motion_model import MotionModel

class Localize:
    def __init__(self, ax, field, max_particles, random, debug:bool) -> None:

        # these will mostly be ROS parameters later.
        self.max_particles = max_particles
        self.particles = np.zeros((0, 3))
        self.field = field # for generating predicted goal headings
        self.ax = ax  # ax reference for plotting
        self.weights = np.array([])
        self.quiver = None # for drawing particles
        self.debug = debug
        self.particle_errors = np.array([])
        self.random = False # inject random particles when resampling
        self.force_resample = False
        self._cbar = None
        self.penalty = 90.0

        self.motion = MotionModel(0.01, 0.02, 0.02, 0.02, 0.01)

        # maybe get rid of later
        # default noise parameters: (x_sigma, y_sigma, heading_sigma_deg)
        self.motion_noise = (0.05, 0.05, 2.0)

        # sensor model angular sigma in degrees (maybe get rid of later...)
        # should come from YOLO model testing
        self.sensor_sigma_deg = 20

        # store estimated pose arrow
        self._pose_arrow = None  

    def _render_particles(self): # should be moved to a plotting class or something
        if self.quiver is not None:
            try:
                self.quiver.remove()
            except Exception:
                pass
            self.quiver = None
        
        if self._cbar is not None:
            try:
                self._cbar.remove()
            except Exception:
                pass
            self._cbar = None

        if self.particles is None or len(self.particles) == 0:
            return

        x = self.particles[:, 0]
        y = self.particles[:, 1]
        th = self.particles[:, 2]
        rad = np.radians(th)
        dx = np.cos(rad) * 0.4
        dy = np.sin(rad) * 0.4

        w = self.weights
        # norm = mcolors.Normalize(vmin=w.min(), vmax=w.max() if w.max() > w.min() else w.min() + 1e-9)
        
        self.quiver = self.ax.quiver(y, x, dy, dx, w,  
                                     scale=1, scale_units='xy', angles='xy', 
                                     alpha=0.8, cmap='plasma')
        
        divider = make_axes_locatable(self.ax)
        cax = divider.append_axes("right", size="3%", pad=0.1)
        self._cbar = self.ax.figure.colorbar(self.quiver, cax=cax)
        self._cbar.set_label('particle weight')

        try:
            self.ax.figure.canvas.draw_idle()
        except Exception:
            pass
    
    @staticmethod
    def _resample(weights):
        '''resamples particles based on normalized weights from weight_particles'''

        n = len(weights)
        w =  np.array(weights, dtype=float)

        c = np.cumsum(w)
        u_0 = np.random.random() / n
        intervals = u_0 + np.arange(n) / n

        return np.searchsorted(c, intervals) # returns indices


    def initial_draw(self, num_particles):
        # generate rand locations and headings.
        x_particles = np.random.uniform(-6, 6, num_particles)
        y_particles = np.random.uniform(-6, 6, num_particles)
        th_particles = np.random.uniform(-180, 180, num_particles)

        # store in nd array type, vstack to align with np.zeros((0,3))
        self.particles = np.vstack([x_particles, y_particles, th_particles]).T
        self.weights = np.full(len(self.particles), 1.0 / len(self.particles))

        if self.debug == True:
            print(f"Plotting {len(self.particles)} particles") 
            print(f"ax object: {self.ax}")
            print(f"x range: {x_particles.min():.2f} to {x_particles.max():.2f}")
            print(f"y range: {y_particles.min():.2f} to {y_particles.max():.2f}")


    def add_noise(self, particles, nm_noise=(0.05, 0.05, 5.0)):
        '''add noise to the particles. Could either be from odometry update or not moving default value (s)'''
        if particles is None or len(particles) == 0:
            return np.zeros((0, 3))
    
        noise = np.array(particles, copy=True)
        noise[:, 0] += np.random.normal(0, nm_noise[0], len(noise))
        noise[:, 1] += np.random.normal(0, nm_noise[1], len(noise))
        noise[:, 2] += np.random.normal(0, nm_noise[2], len(noise))
        noise[:, 2] = np.array([wrap_angle(a) for a in noise[:, 2]])

        return noise


    def estimated_pose(self):
        '''estimate the current pose through averaging all particle x, y, th, and weight'''

        w_norm = self.weights / np.sum(self.weights) # normaized weights
        x = np.average(self.particles[:, 0], weights=w_norm)
        y = np.average(self.particles[:, 1], weights=w_norm)

        th_rad = np.deg2rad(self.particles[:, 2])
        sin_mean = np.average(np.sin(th_rad), weights=w_norm)
        cos_mean = np.average(np.cos(th_rad), weights=w_norm)
        th = math.degrees(math.atan2(sin_mean, cos_mean))
        th = wrap_angle(th)

        pose = (x, y, th)

        if self.debug:
            print(f'estimated pose: {pose}')

        # remove previous single pose arrow (if present)
        if self._pose_arrow is not None:
            try:
                self._pose_arrow.remove()
            except Exception:
                pass
            self._pose_arrow = None

        # draw pose estimate arrow on plot
        rad = np.radians(th)
        dx = np.cos(rad) * 0.4
        dy = np.sin(rad) * 0.4

        self._pose_arrow = self.ax.arrow(
            y, x,
            dy, dx,
            head_width=0.2,
            head_length=0.3,
            fc='red',
            ec='red'
        )

        # redraw
        try:
            self.ax.figure.canvas.draw_idle()
        except Exception:
            pass

        return pose
    
    def calc_error(self, expected, observed):
        '''compares sensor (observed) reading to particle (expected)'''

        # restructuring into dictionaries
        expected_by_color = {}
        observed_by_color = {}

        for color, angle in expected:
            expected_by_color.setdefault(color, []).append(angle)

        for color, angle in observed:
            observed_by_color.setdefault(color, []).append(angle)

        # error:
        # E = sum(|Δθ_i|) + (90 * M) / N 
        #
        # where:
        # Δθ_i = _wrap_angle_deg(θ_obs_i - θ_exp_i) (total_error)
        # N = # of matched angle comparisons (count)
        # M = # of missing or extra detections (count_diff)

        total_error = 0.0
        count = 0 

        for color in expected_by_color.keys() | observed_by_color.keys(): # combine into common colors list (i.e. [0, 1] | [1, 2] = [0, 1, 2])
            expected_angles = sorted(expected_by_color.get(color, []))
            observed_angles = sorted(observed_by_color.get(color, []))

            for expected_angle, observed_angle in zip(expected_angles, observed_angles): # loop through each goal of the same color's angles
                total_error += abs(ang_diff(observed_angle, expected_angle))
                count += 1

            count_diff = abs(len(expected_angles) - len(observed_angles)) 
            total_error += count_diff * self.penalty # penalize missing incorrect number of goals by 70
            count += count_diff # add penalty to total error count

        if count == 0: # avoid division by zero
            return float('inf')

        return total_error / count

    def weight_particles(self, observation): # observation = sensor reading (camera)
        '''
        - weight (and normalize) each particle based on error calculation
        - uses gaussian distribution for weighting
        - fills self.weights instead of returning anything
        '''
        # w = exp ^ (-error^2 / (2 * sigma^2)) <-- gaussian dist.

        n = len(self.particles)
        weights = np.zeros(n)
        errors = np.zeros(n)

        for i, particle in enumerate(self.particles):
            prediction = self.field.predicted_output(particle, 120, 12)
            error = self.calc_error(prediction, observation)
            errors[i] = error
            weights[i] = np.exp(-(error ** 2) / (2 * self.sensor_sigma_deg ** 2))

        self.particle_errors = errors

        total = weights.sum()

        # normalize the weights
        if total > 0:
            weights /= total
        else:
            weights[:] = 1.0 / n
        
        # make available to other functions
        self.weights = weights  
    
    def calc_covariance(self):
        '''calculate the covariance matrix for particles (x, y, th)'''
        x = self.particles[:, 0]
        y = self.particles[:, 1]
        th = self.particles[:, 2]

        data = np.column_stack([x, y, th])
        cov = np.cov(data, rowvar=False) 

        return cov
    
    def update(self, odom: tuple, sensor_reading:list):
        '''update step, run each time a new sensor reading is read'''

        if odom is None or odom == (0.0, 0.0, 0.0):
            moved_particles = self.add_noise(self.particles)
        else:
            moved_particles = self.motion.omnidirectional(odom, self.particles)

        if self.debug:
            delta = moved_particles - self.particles

            th_delta = np.array([
                ang_diff(moved_particles[i, 2], self.particles[i, 2])
                for i in range(len(self.particles))
            ])

            avg_dx = np.mean(delta[:, 0])
            avg_dy = np.mean(delta[:, 1])
            avg_dth = np.mean(th_delta)

            avg_abs_dx = np.mean(np.abs(delta[:, 0]))
            avg_abs_dy = np.mean(np.abs(delta[:, 1]))
            avg_abs_dth = np.mean(np.abs(th_delta))

            print(
                f"odom delta mean: dx={avg_dx:.2f}, dy={avg_dy:.2f}, dth={avg_dth:.2f} deg"
            )
            print(
                f"odom delta mean abs: dx={avg_abs_dx:.2f}, dy={avg_abs_dy:.2f}, dth={avg_abs_dth:.2f} deg"
            )

        self.particles = moved_particles

        # update self.weights
        self.weight_particles(sensor_reading)

        if len(self.weights) == 0: # error handling
            print("no weights available")
            return None
        
        # determine if resampling should occur
        n_eff = 1.0 / np.sum(self.weights ** 2)
        resample = n_eff < (0.7 * len(self.weights))
        
        # resampling
        if resample or self.force_resample:
            idx = self._resample(self.weights)
            self.particles = self.particles[idx]

            # keep most particles, draw random ones based on the covariance 
            if self.random:
                n_random = int(0.1 * len(self.particles))
                rand_x = np.random.uniform(-6, 6, n_random)
                rand_y = np.random.uniform(-6, 6, n_random)
                rand_th = np.random.uniform(-180, 180, n_random)
                random_particles = np.vstack([rand_x, rand_y, rand_th]).T

                self.particles[:n_random] = random_particles
            self.particles = self.add_noise(self.particles)
            # self.weights = np.full(len(self.particles), 1.0 / len(self.particles)) # resets weights to uniform dist?
        
        # rerender matplotlib visualizer
        self._render_particles()

        # return estimated pose
        return self.estimated_pose()

