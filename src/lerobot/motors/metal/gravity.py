"""Pinocchio-based gravity + coriolis + friction feedforward for the metal arm.
Replaces the vendor KDL black box; pinocchio is lazy-imported so the module stays
importable without it (required for CLI startup)."""

GRAVITY_COE = [1.2, 1.15, 1.1, 1.15, 1.0, 1.0]
CORIOLIS_COE = [1.1, 1.15, 1.0, 1.0, 1.1, 1.1]
VISCOUS_COE = [0.15, 0.3025, 0.52, 0.42, 0.015, 0.015]
# per-joint velocity clamps for the viscous term (kdl_solver.cpp ComputeFriction)
_VEL_CLAMP = {2: 0.8, 3: 1.2}


class MetalGravityModel:
    def __init__(self, urdf_path: str):
        import numpy as np
        import pinocchio as pin

        self._np = np
        self._pin = pin
        self.model = pin.buildModelFromUrdf(urdf_path)
        if self.model.nq < 6:
            raise ValueError(f"metal URDF has nq={self.model.nq}, expected >= 6 revolute joints")
        self.data = self.model.createData()

    def _gravity_raw(self, q_rad, gravity_z: float = -9.81):
        np, pin = self._np, self._pin
        q = np.asarray(q_rad[:6], dtype=float)
        model = self.model
        model.gravity.linear = np.array([0.0, 0.0, gravity_z])
        return pin.computeGeneralizedGravity(model, self.data, q)[:6].tolist()

    def _coriolis_raw(self, q_rad, dq_rad):
        np, pin = self._np, self._pin
        q = np.asarray(q_rad[:6], dtype=float)
        dq = np.asarray(dq_rad[:6], dtype=float)
        coriolis = pin.computeCoriolisMatrix(self.model, self.data, q, dq)
        return (coriolis @ dq)[:6].tolist()

    def _friction(self, dq_rad):
        out = []
        for i in range(6):
            v = dq_rad[i]
            if i in _VEL_CLAMP:
                c = _VEL_CLAMP[i]
                v = max(-c, min(c, v))
            out.append(v * VISCOUS_COE[i])
        return out

    def feedforward_torque(self, q_rad, dq_rad):
        g = self._gravity_raw(q_rad)
        c = self._coriolis_raw(q_rad, dq_rad)
        f = self._friction(dq_rad)
        return [GRAVITY_COE[i] * g[i] + CORIOLIS_COE[i] * c[i] + f[i] for i in range(6)]
