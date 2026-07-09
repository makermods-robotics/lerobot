"""Stage 1: follower single-joint sanity (run on hardware, follower on can0).
Reads observations, then nudges joint6 by +3 deg and back. Torque is enabled — keep clear."""
import time
from lerobot.robots.metal_follower.config_metal_follower import MetalFollowerConfig
from lerobot.robots.metal_follower.metal_follower import MetalFollower

r = MetalFollower(MetalFollowerConfig(port="can0"))
r.connect()
try:
    obs = r.get_observation()
    print("observation:", {k: round(v, 2) for k, v in obs.items() if k.endswith(".pos")})
    j6 = obs["joint6.pos"]
    for target in (j6 + 3.0, j6):          # small nudge and back
        r.send_action({"joint6.pos": target})
        time.sleep(0.8)
        print("joint6 ->", round(r.get_observation()["joint6.pos"], 2))
finally:
    r.disconnect()
