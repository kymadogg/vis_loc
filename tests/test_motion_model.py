import pytest
import numpy as np

from vis_loc.motion_model import MotionModel

@pytest.fixture
def model(seed=None):
    return MotionModel(0.5, 0.5, 0.5, 0.5, 0.5, seed)


def test_constructor():
    model =  MotionModel(0.5, 0.5, 0.5, 0.5, 0.5, None)

    assert isinstance(model, MotionModel)


@pytest.mark.parametrize(
    "attr_name, exp_type",
    [
        ("alpha_1", float),
        ("alpha_2", float),
        ("alpha_3", float),
        ("alpha_4", float),
        ("alpha_5", float)
    ]
)
def test_attribute_types(model, attr_name, exp_type):
    attr_value = getattr(model, attr_name)
    
    assert isinstance(attr_value, exp_type)


def test_omnidirectional_without_noise():
    model = MotionModel(0.0, 0.0, 0.0, 0.0, 0.0, seed=1)
    particles = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 90.0]
    ])

    updated = model.omnidirectional((2.0, 3.0, 45.0), particles)

    expected = np.array([
        [2.0, 3.0, 45.0],
        [3.0, 5.0, 135.0]
    ])

    np.testing.assert_allclose(updated, expected)


def test_differential_without_noise():
    model = MotionModel(0.0, 0.0, 0.0, 0.0, 0.0, seed=1)
    particles = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 90.0]
    ])

    updated = model.differential((2.0, 0.0, 45.0), particles)

    expected = np.array([
        [2.0, 0.0, 45.0],
        [3.0, 2.0, 135.0]
    ])

    np.testing.assert_allclose(updated, expected)


def test_random_seed_reproduces_motion_noise():
    particles = np.array([
        [0.0, 0.0, 0.0],
        [1.0, 2.0, 90.0]
    ])
    delta = (2.0, 1.0, 30.0)

    first = MotionModel(0.5, 0.5, 0.5, 0.5, 0.5, seed=123)
    second = MotionModel(0.5, 0.5, 0.5, 0.5, 0.5, seed=123)
    third = MotionModel(0.5, 0.5, 0.5, 0.5, 0.5, seed=456)

    first_result = first.omnidirectional(delta, particles)
    second_result = second.omnidirectional(delta, particles)
    third_result = third.omnidirectional(delta, particles)

    np.testing.assert_allclose(first_result, second_result)
    assert not np.allclose(first_result, third_result)


def test_differential_zero_translation():
    model = MotionModel(0.0, 0.0, 0.0, 0.0, 0.0, seed=1)
    particles = np.array([
        [1.0, 2.0, 90.0]
    ])

    updated = model.differential((0.0, 0.0, 45.0), particles)

    expected = np.array([
        [1.0, 2.0, 135.0]
    ])

    np.testing.assert_allclose(updated, expected)
