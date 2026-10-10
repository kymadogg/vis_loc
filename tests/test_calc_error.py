import unittest

from vis_loc.field import Field
from vis_loc.localize import Localize

field = Field()
loc = Localize(100, 90)


#     # tests i should write at some point...
#         # small vs larger angle offset
#         # same exact pose (error = 0)
#         # different goal, same color, same xy coordinates
#         # penalty behavior?


class TestCalcError(unittest.TestCase):

    def test_same_pose(self):
        coords = (2.0, 2.0)
        obs = field.predicted_output((coords[0], coords[1], 0), 120, 12)
        p = field.predicted_output((coords[0], coords[1], 0), 120, 12)

        assert loc.calc_error(p, obs) == 0

    # small angular error
    def test_angular_error(self):
        coords = (2.0, 2.0)
        obs = field.predicted_output((coords[0], coords[1], 0), 120, 12)
        p_1 = field.predicted_output((coords[0], coords[1], 10), 120, 12)
        p_2 = field.predicted_output((coords[0], coords[1], 15), 120, 12)

        assert loc.calc_error(p_1, obs) < loc.calc_error(p_2, obs)

    def test_different_location_same_obs(self):
        obs = field.predicted_output((2, 2, 0), 120, 12)
        p = field.predicted_output((-2.0, -2.0, 270), 120, 12)
        
        assert loc.calc_error(p, obs) == 0
 
if __name__ == '__main__':
    unittest.main()