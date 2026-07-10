"""Zero-point / direction alignment check (run before recording data).

Reads the leader (can1) and follower (can0) joint angles side by side so you can verify the two
arms agree at the same physical pose. NEITHER arm is torque-enabled, so both stay free to move by
hand. Refreshes an in-place table.

How to read it:
  1. Move BOTH arms to the SAME physical configuration (e.g. both straight up, or both folded the
     same way). Hold them there.
  2. For each joint the leader (L) and follower (F) should read ~the same -> `diff` ≈ 0.
       * a constant non-zero `diff` on a joint  -> ZERO OFFSET  (fix with a per-motor homing_offset)
       * move one joint the same physical way on both arms and L and F change in OPPOSITE
         directions -> DIRECTION REVERSED for that joint (sign/axis mismatch)
  3. Any joint with a large or growing diff pollutes recorded data -> fix before recording.

Usage: python docs/metal/tests/zero_align_check.py [follower_port=can0] [leader_port=can1]
"""
import sys
import time

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.metal import METAL_MOTOR_CONFIG

FPORT = sys.argv[1] if len(sys.argv) > 1 else "can0"
LPORT = sys.argv[2] if len(sys.argv) > 2 else "can1"
NAMES = list(METAL_MOTOR_CONFIG)


def build_bus(port):
    motors = {}
    for name, (send, recv, typ) in METAL_MOTOR_CONFIG.items():
        m = Motor(send, typ, MotorNormMode.DEGREES)
        m.recv_id = recv
        m.motor_type_str = typ
        motors[name] = m
    return DamiaoMotorsBus(port=port, motors=motors, use_can_fd=False, bitrate=1_000_000, can_interface="socketcan")


foll = build_bus(FPORT)
lead = build_bus(LPORT)
foll.connect()  # connect() does NOT enable torque -> the arm stays free to move by hand
lead.connect()

print(f"follower={FPORT}  leader={LPORT}. Pose BOTH arms the same and compare. Ctrl-C to stop.\n")
rows = len(NAMES) + 1
first = True
try:
    while True:
        f = foll.sync_read("Present_Position")
        l = lead.sync_read("Present_Position")
        if not first:
            sys.stdout.write(f"\033[{rows}A")  # move cursor up to overwrite the table in place
        first = False
        print(f"{'joint':8} {'leader':>9} {'follower':>9} {'diff(L-F)':>10}")
        for n in NAMES:
            print(f"{n:8} {l[n]:>9.1f} {f[n]:>9.1f} {l[n] - f[n]:>10.1f}")
        time.sleep(0.3)
except KeyboardInterrupt:
    pass
finally:
    foll.disconnect()
    lead.disconnect()
    print("\nstopped.")
