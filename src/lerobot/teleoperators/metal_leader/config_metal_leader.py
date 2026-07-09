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

from dataclasses import dataclass

from ..config import TeleoperatorConfig


@TeleoperatorConfig.register_subclass("metal_leader")
@dataclass
class MetalLeaderConfig(TeleoperatorConfig):
    """
    Configuration for the Metal arm leader teleoperator: 7 Damiao motors (6 joints + a permanent
    gripper) over classic CAN, driven via the stock `DamiaoMotorsBus`.

    Unlike OpenArms, the gripper is not optional (no `arm_end_type`): the leader always exposes all
    7 motors, all normalized in degrees. A background thread streams Pinocchio gravity-compensation
    MIT torque so the arm feels weightless while a human guides it; the gripper is left backdrivable
    (kp=0, torque=0) so it can be squeezed freely.
    """

    # CAN interface (e.g. "can1"). Linux: "can0", "can1", etc. The leader lives on the second bus.
    port: str = "can1"

    # CAN interface type: "socketcan" (Linux), "slcan" (serial), or "auto" (auto-detect)
    can_interface: str = "socketcan"

    # Metal uses classic CAN @ 1 Mbps (not CAN FD)
    can_bitrate: int = 1_000_000
    use_can_fd: bool = False
    can_data_bitrate: int | None = None

    # Path to the URDF describing the arm. Defaults to the bundled metal_with_gripper.urdf
    # (resolved lazily in MetalLeader.__init__ if left empty).
    urdf_path: str = ""

    # Background gravity-compensation thread rate (Hz).
    gravity_hz: int = 200

    # MIT damping gain applied while the leader is gravity-compensated (kp is always 0, so the
    # human can freely position the arm; kd supplies velocity damping / feel).
    leader_kd: float = 0.3
