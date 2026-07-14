"""Stage 1: follower single-joint sanity (run on hardware, follower on can0).
Reads observations, then nudges wrist_roll by +3 deg and back. Torque is enabled — keep clear."""

import time
from lerobot.robots.metal_follower.config_metal_follower import MetalFollowerConfig
from lerobot.robots.metal_follower.metal_follower import MetalFollower

r = MetalFollower(MetalFollowerConfig(port="can1"))
r.connect()
try:
    obs = r.get_observation()
    print("observation:", {k: round(v, 2) for k, v in obs.items() if k.endswith(".pos")})
    j6 = obs["wrist_roll.pos"]
    for target in (j6 + 3.0, j6):  # small nudge and back
        r.send_action({"wrist_roll.pos": target})
        time.sleep(0.8)
        print("wrist_roll ->", round(r.get_observation()["wrist_roll.pos"], 2))
finally:
    r.disconnect()
