import math
import numpy as np
import numpy.typing as npt

from vis_loc.utilities import wrap_angle, ang_diff
from vis_loc.field import Field
from vis_loc.motion_model import MotionModel

# to-do list for this class
    # restructure particles variable to include weights
        # opt 1: make a dataclass / struct
        # opt 2: add a float element to the NDArray

class Localize:
    def __init__(self, max_particles:int, penalty:int, debug=False) -> None:
        
        # defined when class is intialized
        self.max_particles:int = max_particles
        self.penalty:int = penalty
        self.debug:bool = debug

        # should be added to configuration later
        self.random:bool = False # inject random particles when resampling
        self.force_resample:bool = False
        self.sensor_sigma_deg:int = 20 # sensor angular sigma in degrees, used for weighting particles
        self.model:str = "omni" # TODO: actually implement in update() 

        # other classes
        self.motion = MotionModel(0.01, 0.02, 0.02, 0.02, 0.01) # TODO: make separate variables
        self.field = Field() # for generating predicted goal headings

        # initializing various components 
        self.particles:npt.NDArray = np.zeros((0, 3))
        self.weights:npt.NDArray = np.array([])
        self.particle_errors:npt.NDArray = np.array([])

    
    @staticmethod
    def _resample(weights:npt.NDArray) -> npt.NDArray:
        '''
        resamples particles based on normalized weights

        Args:
            weights: the weights for each particle

        Returns:
            array of n indices representing the resampled particles
        
        1. mapping particle weights onto a continuous cumulative sum line
        2. generates a single random starting point to lay down a uniform grid of n sampling pointers
        3. returns low-variance set of particle indices where heavier-weighted particles are preferred
        '''

        n = len(weights)
        w =  np.array(weights, dtype=float)

        c = np.cumsum(w)
        u_0 = np.random.random() / n
        intervals = u_0 + np.arange(n) / n

        # https://numpy.org/doc/stable/reference/generated/numpy.searchsorted.html#numpy-searchsorted
        resampled = np.searchsorted(c, intervals) 

        return resampled 


    # TODO: make this the sole noise generating function
    # either move it to update(), or add a method to MotionModel for no motion
    @staticmethod
    def _add_noise(particles, static_noise=(0.05, 0.05, 5.0)):
        '''
        Add noise to the particles. Could either be from odometry update or not moving (service call?)
        
        Args:
            particles: current simulated potential poses
            static_noise: the noise values to apply when there is no motion
        '''

        # TODO: confirm wtf static_noise (prev. nm_moise) is
        if particles is None or len(particles) == 0:
            return np.zeros((0, 3))
    
        noise = np.array(particles, copy=True)
        noise[:, 0] += np.random.normal(0, static_noise[0], len(noise))
        noise[:, 1] += np.random.normal(0, static_noise[1], len(noise))
        noise[:, 2] += np.random.normal(0, static_noise[2], len(noise))
        noise[:, 2] = np.array([wrap_angle(a) for a in noise[:, 2]])

        return noise


    def initial_draw(self, num_particles):
        '''
        uniformly generate a number of particles which include
        a (x,y) location and a heading

        Args:
            num_particles: number of particles to generate
        '''

        # TODO: add setting for fixed random seed for later testing & benchmarking

        # generate rand locations and headings
        x_particles = np.random.uniform(-6, 6, num_particles)
        y_particles = np.random.uniform(-6, 6, num_particles)
        th_particles = np.random.uniform(-180, 180, num_particles)

        # store in nd array type, vstack to align with np.zeros((0,3))
        self.particles = np.vstack([x_particles, y_particles, th_particles]).T
        self.weights = np.full(len(self.particles), 1.0 / len(self.particles))

        if self.debug == True:
            print(f"Plotting {len(self.particles)} particles") 
            print(f"x range: {x_particles.min():.2f} to {x_particles.max():.2f}")
            print(f"y range: {y_particles.min():.2f} to {y_particles.max():.2f}")


    def estimated_pose(self) -> tuple:
        '''
        estimate the current pose through averaging all particle x, y, th, and weight
        
        Returns:
            pose: estimated pose of the robot
        '''

        w_norm = self.weights / np.sum(self.weights)

        x = np.average(self.particles[:, 0], weights=w_norm)
        y = np.average(self.particles[:, 1], weights=w_norm)

        th_rad = np.deg2rad(self.particles[:, 2])
        sin_mean = np.average(np.sin(th_rad), weights=w_norm)
        cos_mean = np.average(np.cos(th_rad), weights=w_norm)

        th = math.degrees(math.atan2(sin_mean, cos_mean))
        th = wrap_angle(th)

        pose = (x, y, th) # NOTE: 'th' uses degrees

        if self.debug:
            print(f'estimated pose: {pose}')

        return pose

    
    def calc_error(self, expected, observed) -> float:
        '''
        determines error by compairing sensor reading to a simulated pose

        Args:
            expected: the particle 
            observed: what the camera observes (what robot sees)

        Returns:
            error: the error between the simulated data and observed sensor reading

        scoring function: error = sum(|Δθ_i|) + (90 * M) / N

        where:
        Δθ_i = ang_diff(θ_obs_i - θ_exp_i) --> (d_th)
        N = # of matched angle comparisons 
        M = # of missing or extra detections
        '''

        # restructuring into dictionaries
        exp_by_color = {}
        obs_by_color = {}

        # NOTE: i feel like there is a better way to do this...
        for color, angle in expected:
            exp_by_color.setdefault(color, []).append(angle)

        for color, angle in observed:
            obs_by_color.setdefault(color, []).append(angle)

        # initialize counters
        d_th, N, M = 0.0, 0, 0 

        # combine into common colors list (i.e. [0, 1] | [1, 2] = [0, 1, 2])
        for color in exp_by_color.keys() | obs_by_color.keys(): 
            expected_angles = sorted(exp_by_color.get(color, []))
            observed_angles = sorted(obs_by_color.get(color, []))

            # loop through each goal of the same color's angles
            for expected_angle, observed_angle in zip(expected_angles, observed_angles):
                d_th += abs(ang_diff(observed_angle, expected_angle)) # NOTE: ang_diff() returns degrees!
                N += 1 # add one for each correct color match between expected and observed

            M += abs(len(expected_angles) - len(observed_angles)) # difference in observed/expected number of goals 

        if N == 0: # avoid division by zero
            return float('inf')

        error = (d_th + (self.penalty * M)) / N

        return error


    def weight_particles(self, observation) -> npt.NDArray:
        '''
        weight each particle based on error calculation

        Args:
            observation: sensor reading (camera)

        Returns:
            weights: particle weights

        uses gaussian distribution for weighting 
        w = exp ^ (-error^2 / (2 * sigma^2))
        '''

        # create weights and errors arrays
        n = len(self.particles)
        weights = np.zeros(n)
        errors = np.zeros(n)

        for i, particle in enumerate(self.particles):
            prediction = self.field.predicted_output(particle, 120, 12)
            error = self.calc_error(prediction, observation)
            errors[i] = error

            # gaussian distribution (NOTE: could have numpy function?)
            weights[i] = np.exp(-(error ** 2) / (2 * self.sensor_sigma_deg ** 2))

        self.particle_errors = errors
        total = weights.sum()

        # TODO: see if there is a numpy function that just does this
        # normalize the weights 
        if total > 0:
            weights /= total
        else:
            weights[:] = 1.0 / n
        
        return weights

    
    def calc_covariance(self) -> npt.NDArray:
        '''
        calculate the covariance matrix for particles using np.cov()

        Returns:
            cov: the covariance matrix
        '''

        # TODO: see if this can be condensed at all...
        x = self.particles[:, 0]
        y = self.particles[:, 1]
        th = self.particles[:, 2]

        data = np.column_stack([x, y, th])

        # NOTE: see how this compares to the ROS message type format
        cov = np.cov(data, rowvar=False) 

        return cov

    
    def update(self, delta_odom: tuple, sensor_reading:list):
        '''
        update each time a new sensor reading is recieved

        Args:
            delta_odom: the difference in odometry since the last update
            sensor_reading: a sensor reading from the camera

        Returns:
            pose: estimated pose
            particles: current estimated locations
        '''

        # motion update to the particles
        if delta_odom is None or delta_odom == (0.0, 0.0, 0.0):
            moved_particles = Localize._add_noise(self.particles)
        else:
            moved_particles = self.motion.omnidirectional(delta_odom, self.particles)

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

            print(f"[odom_delta mean] dx: {avg_dx:.2f} | dy: {avg_dy:.2f} | dth: {avg_dth:.2f} deg")
            print(f"[odom_delta mean abs] dx: {avg_abs_dx:.2f} | dy: {avg_abs_dy:.2f} | dth: {avg_abs_dth:.2f} deg")

        # update instance particle and weight variables
        self.particles = moved_particles
        self.weights = self.weight_particles(sensor_reading)

        if len(self.weights) == 0: # error handling
            print("no weights available")
            return []
        
        # determine if resampling should occur
        n_eff = 1.0 / np.sum(self.weights ** 2)
        resample = n_eff < (0.7 * len(self.weights))
        
        # resampling
        if resample or self.force_resample:
            idx = self._resample(self.weights)
            self.particles = self.particles[idx]

            # keep most particles, draw random ones based on the covariance <-- [NOTE: did i even do that?]
            if self.random:
                n_random = int(0.1 * len(self.particles))
                rand_x = np.random.uniform(-6, 6, n_random)
                rand_y = np.random.uniform(-6, 6, n_random)
                rand_th = np.random.uniform(-180, 180, n_random)
                random_particles = np.vstack([rand_x, rand_y, rand_th]).T

                self.particles[:n_random] = random_particles
            self.particles = Localize._add_noise(self.particles)

            # NOTE: future service call /global_localization below (i think this resets weights to uniform dist?) 
            # self.weights = np.full(len(self.particles), 1.0 / len(self.particles)) 
        
        return self.estimated_pose(), self.particles, self.weights

