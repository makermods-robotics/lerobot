"""Stage 2: leader gravity-compensation float test (run on hardware, leader on can1).
Starts the background gravity thread. ACCEPTANCE: the arm feels weightless and holds
position when released. If a joint SAGS -> raise GRAVITY_COE / add handle mass; if a joint
PUSHES AWAY -> sign/axis mismatch for that joint (URDF axis vs motor direction)."""
import time
from lerobot.teleoperators.metal_leader.config_metal_leader import MetalLeaderConfig
from lerobot.teleoperators.metal_leader.metal_leader import MetalLeader

t = MetalLeader(MetalLeaderConfig(port="can1"))
t.connect()
print("Gravity comp running. Move the arm by hand for ~30 s...")
try:
    for _ in range(30):
        a = t.get_action()
        print({k: round(v, 1) for k, v in a.items() if k.endswith(".pos")})
        time.sleep(1.0)
finally:
    t.disconnect()
