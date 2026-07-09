import math

import pytest

from lerobot.motors.metal.gripper import (
    gripper_friction_torque, norm_to_stroke_mm, rad_to_stroke_mm, stroke_mm_to_norm, stroke_mm_to_rad,
)


def test_gripper_friction_torque():
    assert gripper_friction_torque(0.0) == pytest.approx(0.06)           # stop torque at rest
    assert gripper_friction_torque(1.0) == pytest.approx(0.03 + 0.01)    # coulomb + viscous, +dir
    assert gripper_friction_torque(-1.0) == pytest.approx(-0.03 - 0.01)  # opposite direction
    assert gripper_friction_torque(100.0) == pytest.approx(0.03 + 3.0 * 0.01)  # viscous vel clamp

def test_endpoints():
    assert stroke_mm_to_rad(0.0) == 0.002          # angles_[0]
    assert math.isclose(stroke_mm_to_rad(100.0), 1.97527, abs_tol=1e-4)  # angles_ at 100mm

def test_monotonic_increasing():
    prev = -1.0
    for mm in range(0, 101, 5):
        a = stroke_mm_to_rad(float(mm))
        assert a > prev
        prev = a

def test_roundtrip_mm_to_rad_to_mm_is_close():
    for mm in (0.0, 10.0, 37.0, 80.0):
        back = rad_to_stroke_mm(stroke_mm_to_rad(mm))
        assert abs(back - mm) <= 2.0  # table granularity ~1mm

def test_norm_scaling_uses_80mm_full_scale():
    assert stroke_mm_to_norm(80.0) == 100.0
    assert norm_to_stroke_mm(100.0) == 80.0
