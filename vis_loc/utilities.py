import math
import numpy as np

def euclidean_distance(p1: tuple[float, float], p2: tuple[float, float]) -> float:
    distance = math.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
    return distance

def percent_error(estimate, truth, scale):
    scale = abs(scale) if abs(scale) > 1e-9 else 1.0
    return abs(estimate - truth) / scale * 100.0

def wrap_angle(angle, deg=True):
    if deg:
        return ((angle + 180) % 360) - 180
    else:
        return(angle + np.pi) % (2 * np.pi) - np.pi

def ang_diff(a1, a2, deg=True):
    diff = a1 - a2
    if deg:
        return ((diff + 180) % 360) - 180
    else:
        return(diff + np.pi) % (2 * np.pi) - np.pi 

def average_covariance(covariance):
    cov = np.asarray(covariance)
    non_zero = cov[cov != 0]

    if non_zero.size > 0:
        return non_zero.mean()
    return 0

def liang_barsky(x1, y1, x2, y2, rx, ry, half):
    '''
    liang-barsky algorithm, checks whether the robot-to-goal line passes through a goal-sized rectangle centered at another goal. 

    x1, y1 = robot location (start of line segment)
    x2, y2 = goal location (end of line segment)
    rx, ry = rectangle center
    half = rectangle side length /2
    '''
    
    # direction vector of the segment
    dx = x2 - x1
    dy = y2 - y1
    
    # corner points
    p = [-dx, dx, -dy, dy]
    
    # distances from the segment start point to the four edges
    q = [x1 - (rx - half), (rx + half) - x1, y1 - (ry - half), (ry + half) - y1]

    u1, u2 = 0.0, 1.0
    for pi, qi in zip(p, q):
        if abs(pi) < 1e-12:
            if qi < 0:
                return False, None
        else:
            t = qi / pi
            if pi < 0:
                if t > u2:
                    return False, None
                if t > u1:
                    u1 = t
            else:
                if t < u1:
                    return False, None
                if t < u2:
                    u2 = t
    # intersection exists between u1 and u2
    t_hit = u1 if u1 >= 0 else u2
    if 0.0 <= t_hit <= 1.0:
        xi = x1 + t_hit * dx
        yi = y1 + t_hit * dy
        return True, (xi, yi, t_hit)
    return False, None
    