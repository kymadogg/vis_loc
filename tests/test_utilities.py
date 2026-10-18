import pytest
import numpy as np

from vis_loc.utilities import euclidean_distance, percent_error, wrap_angle, ang_diff, average_covariance, liang_barsky

def test_euclidean_distance():
    a, b = (0,0), (1,0)
    result = euclidean_distance(a, b)
    expected = 1.0
    assert result == expected


@pytest.mark.parametrize("est, truth, scale, exp",[
    (100, 100, 1, 0)
])
def test_percent_error(est, truth, scale, exp):
    result = percent_error(est, truth, scale)
    assert result == exp


@pytest.mark.parametrize("angle, exp, deg",[
    (60, 60, True),
    (0.5, 0.5, False)
])
def test_wrap_angle(angle, exp, deg):
    result = wrap_angle(angle, deg)
    assert result == exp


@pytest.mark.parametrize("a1, a2, deg, exp",[
    (30, 30, True, 0),
    (0.5, 0.5, False, 0)
])
def test_ang_diff(a1, a2, deg, exp):
    result = ang_diff(a1, a2, deg)
    assert result == exp


def test_average_covariance():
    cov_mat = np.asarray([
        [0.0, 0.0, 1.0],
        [0.0, 0.0, 0.0]
    ])

    result_a = average_covariance(cov_mat[0])
    result_b = average_covariance(cov_mat[1])

    assert 1.0 == result_a
    assert 0.0 == result_b


@pytest.mark.parametrize(
    ("segment", "square", "expected_hit", "expected_point"),
    [
        ((-2, 0, 2, 0), (0, 0, 1), True, (-1, 0, 0.25)),
        ((-2, 2, 2, 2), (0, 0, 1), False, None),
        ((0, 0, 2, 0), (0, 0, 1), True, (0, 0, 0)),
        ((-2, 0, 0, 0), (0, 0, 1), True, (-1, 0, 0.5)),
        ((-2, 0, 0, 2), (0, 0, 1), True, (-1, 1, 0.5)),
        ((0, 0, 0, 0), (0, 0, 1), True, (0, 0, 0)),
        ((2, 2, 2, 2), (0, 0, 1), False, None)
    ],
    ids = [
        'crosses_square_x=1',
        'misses_square',
        'starts_inside',
        'ends_inside',
        'corner_collision',
        'zero_length_inside',
        "zero_length_outside"
    ]
)
def test_liang_barsky(segment, square, expected_hit, expected_point):
    x1, y1, x2, y2 = segment
    rx, ry, half = square

    hit, point = liang_barsky(x1, y1, x2, y2, rx, ry, half)

    assert hit is expected_hit
    if expected_point is None:
        assert point is None
    else:
        assert point == pytest.approx(expected_point)




