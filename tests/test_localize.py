import pytest
import numpy as np
import numpy.typing as npt

from vis_loc.field import Field
from vis_loc.localize import Localize
from vis_loc.motion_model import MotionModel

field = Field()

'''testing class set up and configuration with parameters'''

@pytest.fixture # NOTE: maybe make a high and low count instance (i.e. high is > 100 and low is 3)
def loc():
    return Localize(500, 90, True)

def test_constructor():
    loc = Localize(500, 90)

    assert isinstance(loc, Localize)

@pytest.mark.parametrize(
    "attr_name, exp_type",
    [
        ("max_particles", int),
        ("penalty", int),
        ("debug", bool),
        ("random",bool),
        ("force_resample", bool),
        ("sensor_sigma_deg", int),
        ("model", str),
        ("motion", MotionModel),
        ("field", Field),
        # ("particles", npt.NDArray), # FIXME: add specific types to each array (i.e dtype=np.float32)
        # ("weights", npt.NDArray),
        # ("particle_errors", npt.NDArray)
    ]
)

def test_attribute_types(loc, attr_name, exp_type):
    attr_value = getattr(loc, attr_name)
    
    assert isinstance(attr_value, exp_type)


'''testing the initial draw, update, covariance, and weight functions'''

def test_initial_draw_creates_uniform_particles_and_weights():
    localizer = Localize(10, 90)

    localizer.initial_draw(10)

    assert localizer.particles.shape == (10, 3) # check shape
    assert np.all(localizer.particles[:, 0] >= -6) # x range
    assert np.all(localizer.particles[:, 0] <= 6)
    assert np.all(localizer.particles[:, 1] >= -6) # y range
    assert np.all(localizer.particles[:, 1] <= 6)
    assert np.all(localizer.particles[:, 2] >= -180) # heading range
    assert np.all(localizer.particles[:, 2] <= 180)
    assert np.all(localizer.weights == pytest.approx(0.1)) # even weights


def test_estimated_pose_uses_circular_heading_average():
    localizer = Localize(2, 90)
    localizer.particles = np.array([
        [1.0, 2.0, 179.0],
        [3.0, 4.0, -179.0]
    ])

    localizer.weights = np.array([0.25, 0.75])

    pose = localizer.estimated_pose() 

    # should be leaning closer to particles[2]
    assert pose[0] == pytest.approx(2.5)
    assert pose[1] == pytest.approx(3.5)
    assert pose[2] == pytest.approx(-179.5) # should be in between 179 and -179


def test_weight_particles_normalizes_weights_and_records_errors():
    localizer = Localize(2, 90)
    localizer.particles = np.array([
        [0.0, 0.0, 0.0],
        [0.0, 0.0, 20.0]
    ])

    observation = field.predicted_output((0.0, 0.0, 0.0), 120, 12)
    weights = localizer.weight_particles(observation)

    assert weights.sum() == pytest.approx(1.0) # weights need to = 1 since they are normalized
    assert weights[0] > weights[1] # pose[0] is closer to the observed data
    assert localizer.particle_errors[0] == pytest.approx(0.0) # pose[0] is the same as obs
    assert localizer.particle_errors[1] > 0 # should be non-zero


def test_calc_covariance_returns_particle_covariance():
    localizer = Localize(3, 90)
    localizer.particles = np.array([
        [0.0, 1.0, 10.0],
        [1.0, 2.0, 20.0],
        [2.0, 3.0, 30.0]
    ])

    covariance = localizer.calc_covariance()

    expected = np.array([
        [1.0, 1.0, 10.0],
        [1.0, 1.0, 10.0],
        [10.0, 10.0, 100.0]
    ])

    np.testing.assert_allclose(covariance, expected)


def test_update_moves_particles_and_returns_estimated_pose(monkeypatch):
    localizer = Localize(2, 90)
    localizer.particles = np.array([
        [0.0, 0.0, 0.0],
        [2.0, 2.0, 10.0]
    ])

    localizer.weights = np.array([0.5, 0.5])

    def move(delta, particles):
        return particles + np.array(delta)

    def weight_particles(observation):
        return np.array([0.5, 0.5])

    monkeypatch.setattr(localizer.motion, "omnidirectional", move)
    monkeypatch.setattr(localizer, "weight_particles", weight_particles)

    pose, particles, weights = localizer.update((1.0, 2.0, 3.0), [])

    exp_result = np.array([
        [1.0, 2.0, 3.0],
        [3.0, 4.0, 13.0]
    ])

    np.testing.assert_allclose(particles, exp_result)
    assert pose == pytest.approx((2.0, 3.0, 8.0))
    assert weights == pytest.approx([0.5, 0.5])
    

def test_delta_odom_is_zero():
    localizer = Localize(1, 90)
    localizer.particles = np.array([
        [2.0, 2.0, 10.0]
    ])
    pose = (2.0, 2.0, 0.0)
    delta_odom = (0.0, 0.0, 0.0)
    sensor_reading = field.predicted_output(pose, 120, 12)
    new_pose = localizer.update(delta_odom, sensor_reading)
    assert new_pose[0] != pose


'''testing the error function specifically'''

def test_same_pose():
    loc = Localize(500, 90)
    coords = (2.0, 2.0)
    obs = field.predicted_output((coords[0], coords[1], 0), 120, 12)
    p = field.predicted_output((coords[0], coords[1], 0), 120, 12)

    assert loc.calc_error(p, obs) == 0


def test_angular_error():
    loc = Localize(500, 90)
    coords = (2.0, 2.0)
    obs = field.predicted_output((coords[0], coords[1], 0), 120, 12)
    p_1 = field.predicted_output((coords[0], coords[1], 10), 120, 12)
    p_2 = field.predicted_output((coords[0], coords[1], 15), 120, 12)

    assert loc.calc_error(p_1, obs) < loc.calc_error(p_2, obs)


def test_different_location_same_obs():
    loc = Localize(500, 90)
    obs = field.predicted_output((2, 2, 0), 120, 12)
    p = field.predicted_output((-2.0, -2.0, 270), 120, 12)
    assert loc.calc_error(p, obs) == 0

'''debug=True outputs the correct number of lines'''

@pytest.fixture
def debug_true_loc():
    loc = Localize(2, 1, True)
    loc.weights = np.array([0.5, 0.5])
    loc.particles = np.array([
        [5.0, 5.0, 10.0],
        [4.0, 4.0, 10.0]
    ])
    return loc

@pytest.mark.parametrize(
    "function_name, inputs, expected_lines",
    [
        ("initial_draw", (2,), 3),
        ("estimated_pose", (), 1),
        ("update", ((0.1, 0.1, 20), field.predicted_output((5.0, 5.0, 10.0), 120, 12)), 3)
    ]
)
def test_debug_is_true(debug_true_loc, capsys, function_name, inputs, expected_lines):
  
    method = getattr(debug_true_loc, function_name)
    method(*inputs)

    captured = capsys.readouterr()
    lines = captured.out.splitlines()

    assert len(lines) == expected_lines

