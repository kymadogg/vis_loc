import pytest
import numpy as np
import matplotlib.pyplot as plt

from vis_loc.field import Field

# NOTE: goal type codes: center = 0, red = 1, blue = 2, neutral = 3

@pytest.fixture
def field():
    return Field()

def test_constructor(field):
    assert isinstance(field, Field)

@pytest.mark.parametrize(
    "attr_name, exp_type",
    [
        ("goal_size", float),
        ("goal_coords", list)
    ]
)

def test_attribute_types(field, attr_name, exp_type):
    attr_value = getattr(field, attr_name)
    
    assert isinstance(attr_value, exp_type)

@pytest.mark.mpl_image_compare(remove_text=False)
def test_plot_base(field):
    fig, _ = field.plot_base()
    return fig 

@pytest.mark.parametrize(
    "pose, fov, range, output",
    [  
        ((4.0, 0.0, 0), 120, 12, []), # nothing is seen
        ((2.0, 2.0, 0), 120, 12, [(3, 0)]), # one neutral goal straight ahead
        ((2.0, -2.0, 0), 120, 12, [(2, 0)]), # blue goal 
        ((-2.0, 2.0, 180), 120, 12, [(1, 0)]), # red goal
        ((-2.0, 0.0, 0), 60, 6, [(0, pytest.approx(0.0))]), # center goal
        ((1.0, 1.0, 45), 120, 1, []), # range is too short to see something
        ((1.0, 1.0, 45), 120, 10, [(3, pytest.approx(26.56, rel=1e-2)), (3, pytest.approx(-26.56, rel=1e-2))]), # two goals
        ((0.0, 6.0, -135), 30, 12, [(1, pytest.approx(0.0))]), # occlusion
        ((-2.0, -1.0, 30), 50, 10, [(0, pytest.approx(-3.43, rel=1e-2)), (3, pytest.approx(21.34, rel=1e-2))])
    ],
    ids=[
        'no_goals',
        'neutral_goal',
        'blue_goal',
        'red_goal',
        'center_goal',
        'too_short_range',
        'multiple_goals',
        'occlusion',
        'one_occluded_one_not'
    ]
)
def test_predicted_output(field, pose, fov, range, output):
    result = field.predicted_output(pose, fov, range)
    assert result == output
