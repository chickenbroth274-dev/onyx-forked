import pytest

from turret_control.target_localizer import (
    calculate_center,
    calculate_error,
)


def test_calculate_center():
    assert calculate_center(400, 200, 200, 200) == (500.0, 300.0)


def test_calculate_error():
    assert calculate_error(500, 300, 1280, 720) == (-140.0, -60.0)


def test_image_center():
    assert calculate_error(640, 360, 1280, 720) == (0.0, 0.0)


def test_invalid_bounding_box():
    with pytest.raises(ValueError):
        calculate_center(100, 100, 0, 200)


def test_invalid_image_dimensions():
    with pytest.raises(ValueError):
        calculate_error(100, 100, 0, 720)
