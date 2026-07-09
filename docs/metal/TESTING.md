# Metal arm — staged hardware bring-up & testing

> Software (offline) is complete and unit-tested with mocks. The steps below need the **physical
> arm** and are run by the operator. Torque is enabled in several steps — keep the workspace clear
> and be ready to cut power.

## 0. Prerequisites
- Env with the integration installed: `pip install -e ".[metal]"` (pulls python-can + pinocchio).
- Follower arm on **can0**, leader arm on **can1**. Motors powered (24 V), CAN wiring + 120 Ω termination in place.
- Bring the buses up (classic CAN @ 1 Mbps):
  ```
  bash docs/metal/start_can.sh
  ```
  (For the DM-USB2FDCAN USB adapter over slcan instead of a native CAN card, use `slcand` and set `can_interface="slcan"` in the configs.)

## 1. Follower single-joint sanity (can0)
Confirms CAN id map, enable, read-back, and a commanded move.
```
python docs/metal/tests/follower_smoke.py
```
Expect: printed observation for all 7 motors, and `joint6` moving ~+3° then back. If it errors on connect → check CAN up, ids `0x01..0x07`/`0x11..0x17`, power, termination.

## 2. Leader gravity-compensation "float" test (can1) — the key check
```
python docs/metal/tests/leader_float.py
```
**Acceptance: the arm feels weightless and holds position when released.**
- A joint **sags** (drifts down) → gravity underestimated: raise that joint's `GRAVITY_COE` in `src/lerobot/motors/metal/gravity.py`, and/or add the **leader handle mass** to the model (the handle isn't in the URDF yet).
- A joint **pushes away / runs off** → **sign error**: the motor's positive direction disagrees with the URDF joint axis for that joint. Do NOT keep running; fix the sign before proceeding.
- Feels **weightless but drifts/oscillates** → increase `leader_kd` (MIT damping) a little.

## 3. Leader ↔ follower zero alignment
Teleop maps leader joint degrees → follower target degrees 1:1, so the two arms must agree at the same physical pose.
- Put both arms in the same pose; compare `get_observation()` (follower) and `get_action()` (leader) per joint.
- If they differ, set a per-motor `homing_offset` via the bus calibration (`MotorCalibration`) and persist it, until both read the same at a shared reference pose.

## 4. End-to-end teleoperation
```
lerobot-teleoperate \
  --robot.type=metal_follower --robot.port=can0 \
  --teleop.type=metal_leader  --teleop.port=can1
```
Expect: follower tracks the leader; squeezing the leader gripper drives the follower gripper. Record a short clip for the upstream PR (strong social signal). Then record a dataset with `lerobot-record` (see `docs/source/metal.mdx`).

## Known tuning items (deferred from the offline build)
- **Follower gains**: currently the DamiaoMotorsBus defaults (kp=10, kd=0.5) → soft tracking. Tune toward the vendor "follow" gains (J1 200/3, J2 500/5, J3 400/5, J4 200/2, J5/J6 20/0.1 — from `y1_sdk .../config/motor_config.cpp`). Set per-motor via `bus.sync_write("Kp", {...})` / `bus.sync_write("Kd", {...})`, verifying the MIT kp/kd range matches the vendor's before trusting absolute values.
- **Gripper units**: passed through as raw motor degrees today (correct for identical-arm teleop). To expose physical 0–100 stroke for datasets/policies, wire in the bundled nonlinear table (`src/lerobot/motors/metal/gripper.py`).
- **Leader handle mass**: not in the gravity model yet; add it (separate URDF or end-mass offset) if Step 2 shows end-effector sag.
- **Adapter/throughput**: native SocketCAN preferred for the 7-motor loop rate; verify slcan throughput if using the USB adapter.
