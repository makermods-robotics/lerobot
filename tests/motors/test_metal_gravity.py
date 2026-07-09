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
