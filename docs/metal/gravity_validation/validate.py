"""Compare Pinocchio (shipping engine) against the vendor's KDL on identical inputs.
Pinocchio side runs in the metal-lerobot env; KDL side is the compiled kdl_oracle
(built against ROS Humble). Prints max/mean abs error for gravity and coriolis."""
import subprocess, numpy as np, pinocchio as pin

URDF = "src/lerobot/motors/metal/urdf/metal_with_gripper.urdf"
ORACLE = "docs/metal/gravity_validation/kdl_oracle"
N = 300

m = pin.buildModelFromUrdf(URDF); d = m.createData()
m.gravity.linear = np.array([0., 0., -9.81])
lims_deg = [(-160,160),(-180,0),(0,180),(-123,81),(-85,85),(-145,145)]
lo = np.radians([a for a,_ in lims_deg]); hi = np.radians([b for _,b in lims_deg])
rng = np.random.default_rng(0)
Q  = rng.uniform(lo, hi, size=(N,6)); Q[0] = 0.0
QD = rng.uniform(-2.0, 2.0, size=(N,6)); QD[0] = 0.0

# Pinocchio: gravity g(q); coriolis C(q,dq)dq = rnea(q,dq,0) - g(q)
pin_g = np.array([pin.computeGeneralizedGravity(m, d, q) for q in Q])
pin_c = np.array([pin.rnea(m, d, q, dq, np.zeros(6)) - pin.computeGeneralizedGravity(m, d, q)
                  for q, dq in zip(Q, QD)])

# KDL oracle
stdin = "\n".join(" ".join("%.15g" % v for v in np.concatenate([q, qd])) for q, qd in zip(Q, QD))
out = subprocess.run([ORACLE, URDF], input=stdin, capture_output=True, text=True, check=True).stdout
K = np.array([[float(x) for x in line.split(",")] for line in out.strip().splitlines()])
kdl_g, kdl_c = K[:, :6], K[:, 6:]

for name, P, Kx in (("GRAVITY g(q)", pin_g, kdl_g), ("CORIOLIS C(q,dq)dq", pin_c, kdl_c)):
    err = np.abs(P - Kx)
    print(f"{name}: max|err|={err.max():.3e} N·m  mean|err|={err.mean():.3e}  "
          f"max|signal|={np.abs(Kx).max():.3f}")
