import unittest
import numpy as np
import matplotlib.pyplot as plt

from field import Field
from utilities import ang_diff

field = Field()

def calc_error(expected, observed, penalty):
    '''compares sensor (observed) reading to particle (expected)'''

    # restructuring into dictionaries
    expected_by_color = {}
    observed_by_color = {}

    for color, angle in expected:
        expected_by_color.setdefault(color, []).append(angle)

    for color, angle in observed:
        observed_by_color.setdefault(color, []).append(angle)

    # error = sum(|Δθ_i|) + (90 * M) / N 

    # where:
    # Δθ_i = ang_diff(θ_obs_i - θ_exp_i) (total_error) <-- also in deg...
    # N = # of matched angle comparisons (count) (how many colors are correct)
    # M = # of missing or extra detections (count_diff)

    total_error = 0.0
    count = 0 

    for color in expected_by_color.keys() | observed_by_color.keys(): # combine into common colors list (i.e. [0, 1] | [1, 2] = [0, 1, 2])
        expected_angles = sorted(expected_by_color.get(color, []))
        observed_angles = sorted(observed_by_color.get(color, []))

        for expected_angle, observed_angle in zip(expected_angles, observed_angles): # loop through each goal of the same color's angles
            total_error += abs(ang_diff(observed_angle, expected_angle))
            count += 1 # increment for each correct goal seen

        count_diff = abs(len(expected_angles) - len(observed_angles)) 
        total_error += count_diff * penalty # penalize missing incorrect number of goals by a fixed value
        count += count_diff # add penalty to total error count

    if count == 0: # avoid division by zero
        return float('inf')

    return total_error / count

angle = []
fn_out = []

coords = (2.0, 2.0) # x, y

for i in range(360):
    obs = field.predicted_output((coords[0], coords[1], 0), 120, 12)
    p = field.predicted_output((coords[0], coords[1], i), 120, 12)
    error = calc_error(p, obs, 90)
    # print(f'angle: {i} | error:{error}')
    fn_out.append(error) 
    angle.append(i)

# tests i should write at somepoint...
    # small vs larger angle offset
    # same exact pose (error = 0)
    # different goal, same color, same xy coordinates
    # penalty behavior?

# debug plotting
plt.plot(angle, fn_out)

plt.xlabel('angle')
plt.ylabel('error')
plt.title('error function behavior, turning in place (x: 2.0, y: 2.0)')

plt.show()
