import numpy as np
from numpy import typing as npt
from vis_loc.utilities import ang_diff, wrap_angle

class MotionModel:
    def __init__(self, a1, a2, a3, a4, a5) -> None:
        self.alpha_1 = a1
        self.alpha_2 = a2
        self.alpha_3 = a3
        self.alpha_4 = a4
        self.alpha_5 = a5

    def differential(self, delta:tuple, particles:npt.NDArray):
        '''motion model for a diff drive robot, adapted from nav2_amcl'''

        dx, dy, dth = delta
        d_tr = np.hypot(dx, dy)

        # dont include small rotations
        if d_tr < 1e-2:
            d_r1 = 0
        else:
            movement_dir = np.degrees(np.arctan2(dy, dx))  # scalar, same for all particles
            d_r1 = ang_diff(movement_dir, particles[:, 2])
        d_r2 = ang_diff(dth, d_r1)

        d_r1_noise = np.minimum(np.abs(ang_diff(d_r1, 0.0)),np.abs(ang_diff(d_r1, 180.0)))
        d_r2_noise = np.minimum(np.abs(ang_diff(d_r2, 0.0)),np.abs(ang_diff(d_r2, 180.0)))

        r1_noise_stddev = np.sqrt(self.alpha_1 * d_r1_noise ** 2 + self.alpha_2 * d_tr ** 2)
        r2_noise_stddev = np.sqrt(self.alpha_1 * d_r2_noise ** 2 + self.alpha_2 * d_tr ** 2)
        tr_noise_stddev = np.sqrt(self.alpha_3 * d_tr ** 2 + self.alpha_4 * d_r1_noise ** 2 + self.alpha_4 * d_r2_noise ** 2)

        d_r1_hat = d_r1 - np.random.normal(0, r1_noise_stddev)
        d_r2_hat = d_r2 - np.random.normal(0, r2_noise_stddev)
        d_tr_hat = d_tr - np.random.normal(0, tr_noise_stddev)

        # convert some stuff to radians for trig
        th_r = np.deg2rad(particles[:,2])
        d_r1_hat_r = np.deg2rad(d_r1_hat)

        x = particles[:, 0] + (d_tr_hat * np.cos(th_r + d_r1_hat_r))
        y = particles[:, 1] + (d_tr_hat * np.sin(th_r + d_r1_hat_r))
        th = particles[:, 2] + (d_r1_hat + d_r2_hat)

        th = np.array([wrap_angle(t) for t in th])

        updated_particles = np.stack([x, y, th], axis=-1)

        return updated_particles
    
  
    def omnidirectional(self, delta, particles:npt.NDArray):
        '''motion model for a omni directional robot, adapted from nav2_amcl'''
        dx, dy, dth = delta

        th_r = np.deg2rad(particles[:,2])

        d_trans = np.hypot(dx, dy)
        d_rot = np.deg2rad(dth)

        # noise distributions
        trans_hat_stddev = np.sqrt(self.alpha_3 * d_trans**2 + self.alpha_4 * d_rot**2)
        rot_hat_stddev = np.sqrt(self.alpha_1 * d_rot**2 + self.alpha_2 * d_trans**2)
        strafe_hat_stddev = np.sqrt(self.alpha_4 * d_rot**2 + self.alpha_5 * d_trans**2)

        # sample noises
        d_trans_hat = d_trans + np.random.normal(0, trans_hat_stddev)
        d_rot_hat = d_rot + np.random.normal(0, rot_hat_stddev)
        d_strafe_hat = np.random.normal(0, strafe_hat_stddev)

        # calc delta heading
        motion_ang = np.arctan2(dy, dx)
        d_heading = ang_diff(motion_ang, th_r, deg=False)
        d_heading += th_r

        c_heading = np.cos(d_heading)
        s_heading = np.sin(d_heading)

        x = particles[:, 0] + (d_trans_hat * c_heading + d_strafe_hat * s_heading)
        y = particles[:, 1] + (d_trans_hat * s_heading - d_strafe_hat * c_heading)
        th = particles[:, 2] + np.rad2deg(d_rot_hat)

        th = np.array([wrap_angle(t) for t in th])

        updated_particles = np.stack([x, y, th], axis=-1)
        return updated_particles

        