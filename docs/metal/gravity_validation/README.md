# Gravity / Coriolis validation — Pinocchio vs vendor KDL

The metal arm's gravity compensation ships as **pure-Python Pinocchio** (`src/lerobot/motors/metal/gravity.py`), replacing the vendor SDK's **KDL** (ROS) dynamics. This directory proves the switch is numerically exact: on identical inputs, Pinocchio and the vendor-exact KDL path produce the same joint torques to floating-point precision.

## Result (300 random joint configs within soft limits, seed 0)

| Term | max \|error\| | mean \|error\| | max \|signal\| | error / signal |
|---|---|---|---|---|
| Gravity `g(q)` | 4.9e-11 N·m | 1.7e-12 | 14.08 N·m | ~3e-12 |
| Coriolis `C(q,q̇)q̇` | 4.9e-12 N·m | 1.5e-13 | 1.97 N·m | ~2e-12 |

The residual is machine-epsilon accumulation from different recursion orders — ~12 orders of magnitude below the signal. **Conclusion: Pinocchio == vendor KDL.**

## Why they match
Both implement the exact RNEA (recursive Newton–Euler) algorithm and read the **same** URDF inertials. Verified model alignment: both use 6 revolute joints `JOINT1..JOINT6`; the gripper is lumped as fixed mass into `Link6` (not a separate branch), so KDL's `getChain("base_link","Link6")` and Pinocchio's full model are the same system; `fixed_base_joint` is identity, so gravity `(0,0,-9.81)` is the same physical direction in both. The vendor's empirical `gravity_coe`/`coriolis_coe`/friction are plain multipliers applied on top of this (now-proven-identical) engine output.

## Files
- `kdl_oracle.cpp` — vendor-exact KDL dumper: `kdl_parser::treeFromFile` → `getChain("base_link","Link6")` → `ChainDynParam::JntToGravity` / `JntToCoriolis` (mirrors `y1_sdk .../kdl_solver.cpp`). Reads `q[6] qd[6]` rows, writes `gravity[6],coriolis[6]`.
- `validate.py` — Pinocchio side + comparison (runs in the `metal-lerobot` env).

## Reproduce
KDL side needs ROS Humble + orocos-kdl (one-off; NOT part of the shipping package or CI). From the repo root:

```
source /opt/ros/humble/setup.bash
g++ docs/metal/gravity_validation/kdl_oracle.cpp -o docs/metal/gravity_validation/kdl_oracle \
  -I/opt/ros/humble/include/kdl_parser -I/opt/ros/humble/include -I/usr/include/kdl \
  -I/opt/ros/humble/include/urdfdom_headers -I/opt/ros/humble/include/urdf -I/usr/include/eigen3 \
  -L/opt/ros/humble/lib -lkdl_parser -lorocos-kdl -lurdf -Wl,-rpath,/opt/ros/humble/lib
python docs/metal/gravity_validation/validate.py   # metal-lerobot env, with ROS libs on LD_LIBRARY_PATH
```

The compiled `kdl_oracle` binary and scratch CSVs are git-ignored; only the sources + this result are tracked.
