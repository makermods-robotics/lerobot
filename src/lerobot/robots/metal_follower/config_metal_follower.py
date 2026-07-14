#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from dataclasses import dataclass, field

from lerobot.cameras import CameraConfig

from ..config import RobotConfig


@RobotConfig.register_subclass("metal_follower")
@dataclass
class MetalFollowerConfig(RobotConfig):
    """
    Configuration for the Metal arm follower robot: 7 Damiao motors (6 joints + a permanent
    gripper) over classic CAN, driven via the stock `DamiaoMotorsBus` in MIT position control.

    Unlike OpenArms, the gripper is not optional (no `arm_end_type`): the arm always exposes all
    7 motors, all normalized in degrees (the gripper is passed through in raw motor degrees, not
    converted via a stroke table).
    """

    # CAN interface (e.g. "can0"). Linux: "can0", "can1", etc.
    port: str = "can0"

    # CAN interface type: "socketcan" (Linux), "slcan" (serial), or "auto" (auto-detect)
    can_interface: str = "socketcan"

    # Metal uses classic CAN @ 1 Mbps (not CAN FD)
    can_bitrate: int = 1_000_000
    use_can_fd: bool = False
    can_data_bitrate: int | None = None

    # Path to the URDF describing the arm. Defaults to the bundled metal_with_gripper.urdf
    # (resolved lazily in MetalFollower.__init__ if left empty).
    urdf_path: str = ""

    # Per-motor MIT follow gains {name: (kp, kd)}. None -> METAL_FOLLOWER_GAINS (vendor values).
    # Set at connect(); the bus default (kp=10) is too soft to hold the arm against gravity.
    gains: dict[str, tuple[float, float]] | None = None

    # Slow initial synchronization: at teleop start the follower can be far from the leader, and
    # firm follow gains would snap it there hard. Until it has caught up, cap each joint's per-step
    # motion to this many degrees (gentle alignment), then track at full speed. None disables it.
    startup_sync_speed_deg: float | None = 1.0
    startup_sync_tolerance_deg: float = 3.0

    # Safety limit for relative target positions (degrees). None disables the check.
    max_relative_target: float | dict[str, float] | None = None

    # Acceleration (jerk) limit: max change in per-step motion (degrees) between consecutive
    # send_action calls. Where max_relative_target caps the step size (velocity), this caps how
    # fast the step itself can change (acceleration), so the arm can't suddenly jerk/accelerate.
    # Smooths starts, stops and policy-output spikes WITHOUT touching kp/kd. None disables it.
    # Deploy-time smoothing; leave None for teleop/recording.
    max_relative_accel_deg: float | None = None

    # Median filter window (frames) applied to each incoming action before anything else. A single-
    # frame spike in the policy output is discarded by the median, while real motion passes through.
    # Best tool against per-frame jitter. 1 disables it; use an odd window like 3 or 5. Adds ~half a
    # frame of latency. Does not touch kp/kd. Deploy-time smoothing; leave 1 for teleop/recording.
    median_filter_window: int = 1

    # Whether to disable torque when disconnecting
    disable_torque_on_disconnect: bool = False

    # Camera configurations
    cameras: dict[str, CameraConfig] = field(default_factory=dict)
