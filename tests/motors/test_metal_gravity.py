import pytest

pin = pytest.importorskip("pinocchio")
from lerobot.motors.metal.gravity import MetalGravityModel  # noqa: E402

URDF = "src/lerobot/motors/metal/urdf/metal_with_gripper.urdf"

@pytest.fixture(scope="module")
def gm():
    return MetalGravityModel(URDF)

def test_returns_six_torques(gm):
    tau = gm.feedforward_torque([0.0]*6, [0.0]*6)
    assert len(tau) == 6

def test_pure_gravity_matches_pinocchio_rnea(gm):
    import numpy as np
    import pinocchio as pin
    q = np.array([0.1, -0.5, 0.7, 0.2, -0.3, 0.4])
    g_ref = pin.rnea(gm.model, gm.data, q, np.zeros(6), np.zeros(6))
    g_raw = gm._gravity_raw(q.tolist())
    assert np.allclose(g_raw, g_ref, atol=1e-9)

def test_gravity_flips_sign_with_gravity_vector(gm):
    import numpy as np
    q = [0.3, -0.4, 0.5, 0.0, 0.2, -0.1]
    up = np.array(gm._gravity_raw(q))
    down = np.array(gm._gravity_raw(q, gravity_z=+9.81))
    assert np.allclose(up, -down, atol=1e-9)

def test_friction_opposes_velocity(gm):
    tau_still = gm.feedforward_torque([0.0]*6, [0.0]*6)
    tau_move = gm.feedforward_torque([0.0]*6, [1.0]*6)
    assert tau_move[2] > tau_still[2]  # joint idx 2 has largest viscous coe (0.52)

def test_teleop_feedforward_deadbands_velocity_noise(gm):
    tmax = [1000.0] * 6  # clamp inert
    still = gm.teleop_feedforward([0.0]*6, [0.0]*6, tmax)
    crawling = gm.teleop_feedforward([0.0]*6, [0.04]*6, tmax)  # below 0.05 rad/s deadband
    moving = gm.teleop_feedforward([0.0]*6, [0.06]*6, tmax)
    assert crawling == pytest.approx(still)
    assert max(abs(a - b) for a, b in zip(moving, still, strict=True)) > 1e-4

def test_teleop_feedforward_clamps_to_half_tmax(gm):
    unclamped = gm.feedforward_torque([0.0]*6, [0.0]*6)
    assert abs(unclamped[2]) > 0.5  # joint3 gravity at zero pose actually engages the clamp
    tau = gm.teleop_feedforward([0.0]*6, [0.0]*6, [1.0]*6)
    assert all(abs(t) <= 0.5 + 1e-12 for t in tau)
    assert tau[2] == pytest.approx(0.5 if unclamped[2] > 0 else -0.5)
