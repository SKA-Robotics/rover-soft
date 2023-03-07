import pytest
from motion_planning import MotionInterpolator


def assert_equals(given, target):
    assert abs(given - target) < 0.001


def assert_equals_list(given, target):
    for givenElement, targetElement in zip(given, target):
        assert_equals(givenElement, targetElement)


def test_assert_equals():
    assert_equals(1, 1.0)
    with pytest.raises(AssertionError):
        assert_equals(1, -1)


def test_assert_equals_list():
    assert_equals_list([1, 2], [1, 2])
    with pytest.raises(AssertionError):
        assert_equals_list([1, 9], [1, 3])


def test_calculate_vector_invalid_size():
    interpolator = MotionInterpolator(1, 1, 0.1)
    with pytest.raises(Exception, match="Cannot compare lists of different sizes!"):
        interpolator._calculate_movement_vector([0, 0, 0, 0], [0, 0, 0])


def test_calculate_vector_one_direction():
    interpolator = MotionInterpolator(1, 1, 0.1)
    vector = interpolator._calculate_movement_vector([0, 0, 0], [1, 0, 0])
    assert_equals_list(vector, [1, 0, 0])


def test_calculate_vector_one_direction_normalized():
    interpolator = MotionInterpolator(1, 1, 0.1)
    vector = interpolator._calculate_movement_vector([0, 0, 0], [333, 0, 0])
    assert_equals_list(vector, [1, 0, 0])


def test_calculate_vector_all_directions():
    interpolator = MotionInterpolator(1, 1, 0.1)
    vector = interpolator._calculate_movement_vector([1, -2], [4, 2])
    assert_equals_list(vector, [3 / 5, 4 / 5])


def test_calculate_distance():
    interpolator = MotionInterpolator(1, 1, 0.1)
    distance = interpolator._calculate_distance([1, -2], [4, 2])
    assert_equals(distance, 5)
