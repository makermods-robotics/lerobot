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

import logging
import math
import time
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.damiao.tables import MOTOR_LIMIT_PARAMS, MotorType
from lerobot.motors.metal import (
    METAL_FOLLOWER_GAINS,
    METAL_JOINT_LIMITS_DEG,
    METAL_JOINT_NAMES,
    METAL_MOTOR_CONFIG,
)
from lerobot.types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..robot import Robot
from ..utils import ensure_safe_goal_position
from .config_metal_follower import MetalFollowerConfig

logger = logging.getLogger(__name__)

# Bundled URDF used by default when `config.urdf_path` is left empty. The gripper is permanent
# on the metal arm, so this is always the "with gripper" variant (no arm_end_type branching).
DEFAULT_URDF_PATH = str(
    Path(__file__).resolve().parents[2] / "motors" / "metal" / "urdf" / "metal_with_gripper.urdf"
)

# Startup-sync stall release: a joint counts as making progress when it moved more than
# _SYNC_PROGRESS_EPS_DEG since its last progress mark; after _SYNC_STALL_RELEASE_SEC without
# progress it is released from the slow sync (it physically cannot close the gap at the capped
# step size, e.g. parked past a soft limit or stiction above kp * step).
_SYNC_PROGRESS_EPS_DEG = 0.2
_SYNC_STALL_RELEASE_SEC = 1.0


class MetalFollower(Robot):
    """
    Metal arm follower robot: 6 joints + a permanent gripper, all driven as Damiao motors over
    classic CAN (`use_can_fd=False`) via the stock `DamiaoMotorsBus` in MIT position control.

    All 7 motors are normalized in degrees. The gripper is passed through in raw motor degrees,
    same as the joints (no stroke-to-angle conversion here; that's deferred to a later task).
    """

    config_class = MetalFollowerConfig
    name = "metal_follower"

    def __init__(self, config: MetalFollowerConfig):
        super().__init__(config)
        self.config = config

        if not config.urdf_path:
            config.urdf_path = DEFAULT_URDF_PATH

        # Build all 7 motors (6 joints + permanent gripper) from METAL_MOTOR_CONFIG.
        motors: dict[str, Motor] = {}
        for motor_name, (send_id, recv_id, motor_type_str) in METAL_MOTOR_CONFIG.items():
            motor = Motor(send_id, motor_type_str, MotorNormMode.DEGREES)
            motor.recv_id = recv_id
            motor.motor_type_str = motor_type_str
            motors[motor_name] = motor

        self._joint_motor_names = list(motors)

        self.bus = DamiaoMotorsBus(
            port=self.config.port,
            motors=motors,
            calibration=self.calibration,
            can_interface=self.config.can_interface,
            use_can_fd=self.config.use_can_fd,
            bitrate=self.config.can_bitrate,
            data_bitrate=self.config.can_data_bitrate if self.config.use_can_fd else None,
        )

        self.cameras = make_cameras_from_configs(config.cameras)

        # False until the follower has caught up to the leader (slow initial sync), then full speed.
        self._synced = False
        self._synced_motors: set[str] = set()
        self._sync_progress: dict[str, tuple[float, float]] = {}
        self._resolved_gains: dict[str, tuple[float, float]] = {}
        self._reset_velocity_feedforward()

        # Built at connect() when config.torque_feedforward is set (needs pinocchio).
        self._torque_ff_model = None
        self._torque_ff_tmax = [
            MOTOR_LIMIT_PARAMS[getattr(MotorType, METAL_MOTOR_CONFIG[name][2].upper())][2]
            for name in METAL_JOINT_NAMES
        ]

    def _reset_velocity_feedforward(self) -> None:
        self._velocity_ff_previous_goal: dict[str, float] | None = None
        self._velocity_ff_velocities: dict[str, float] = {}
        self._velocity_ff_last_action_time: float | None = None

    def _estimate_command_velocities(self, goal_pos: dict[str, float]) -> dict[str, float]:
        now = time.perf_counter()
        if self._velocity_ff_last_action_time is None or now - self._velocity_ff_last_action_time > 0.5:
            self._reset_velocity_feedforward()

        if self._velocity_ff_previous_goal is None:
            velocities = dict.fromkeys(goal_pos, 0.0)
        else:
            elapsed = now - self._velocity_ff_last_action_time
            dt = max(0.005, min(0.1, elapsed))
            velocities = {}
            for motor, position in goal_pos.items():
                previous_position = self._velocity_ff_previous_goal.get(motor)
                if previous_position is None:
                    velocities[motor] = 0.0
                    continue
                raw_velocity = (position - previous_position) / dt
                max_velocity = self.config.velocity_ff_max_deg_s
                raw_velocity = max(-max_velocity, min(max_velocity, raw_velocity))
                previous_velocity = self._velocity_ff_velocities.get(motor, 0.0)
                velocities[motor] = (
                    1.0 - self.config.velocity_ff_alpha
                ) * previous_velocity + self.config.velocity_ff_alpha * raw_velocity

        self._velocity_ff_previous_goal = dict(goal_pos)
        self._velocity_ff_velocities = velocities
        self._velocity_ff_last_action_time = now
        return dict(velocities)

    def _feedforward_torques(self) -> dict[str, float]:
        """Gravity + Coriolis + friction tau_ff (Nm) per joint from the bus state cache
        (updated by every response frame — at most one tick stale, no extra CAN traffic).
        Gripper stays 0, matching the vendor teleop loop."""
        states = self.bus.get_cached_states(METAL_JOINT_NAMES)
        q = [math.radians(states[name]["position"]) for name in METAL_JOINT_NAMES]
        dq = [math.radians(states[name]["velocity"]) for name in METAL_JOINT_NAMES]
        tau = self._torque_ff_model.teleop_feedforward(q, dq, self._torque_ff_tmax)
        torques = dict.fromkeys(self._joint_motor_names, 0.0)
        torques.update(dict(zip(METAL_JOINT_NAMES, tau, strict=True)))
        return torques

    @property
    def _motors_ft(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self._joint_motor_names}

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        features: dict[str, tuple] = {}
        for cam_key, cam in self.cameras.items():
            features[cam_key] = (cam.height, cam.width, 3)
        return features

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        return {**self._motors_ft, **self._cameras_ft}

    @cached_property
    def action_features(self) -> dict[str, type]:
        return self._motors_ft

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected and all(cam.is_connected for cam in self.cameras.values())

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting arm on {self.config.port}...")
        self.bus.connect()

        for cam in self.cameras.values():
            cam.connect()

        self.bus.enable_torque()
        # Re-arm the slow initial sync on every connect.
        self._synced = False
        self._synced_motors: set[str] = set()
        self._sync_progress: dict[str, tuple[float, float]] = {}
        self._reset_velocity_feedforward()

        # Set firm follow gains (bus default kp=10 is far too soft to hold the arm against
        # gravity → the follower sags). Uses vendor follow_mit_kp/kd unless overridden.
        gains = self.config.gains or METAL_FOLLOWER_GAINS
        gains = {m: kpkd for m, kpkd in gains.items() if m in self._joint_motor_names}
        self._resolved_gains = {m: (kp, kd) for m, (kp, kd) in gains.items()}
        self.bus.sync_write("Kp", {m: kp for m, (kp, kd) in self._resolved_gains.items()})
        self.bus.sync_write("Kd", {m: kd for m, (kp, kd) in self._resolved_gains.items()})

        if self.config.torque_feedforward and self._torque_ff_model is None:
            from lerobot.motors.metal.gravity import MetalGravityModel  # noqa: PLC0415

            try:
                self._torque_ff_model = MetalGravityModel(self.config.urdf_path)
            except ImportError as e:
                raise ImportError(
                    "torque_feedforward requires pinocchio; install with `uv sync --extra metal`"
                ) from e

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # The metal arm has no separate calibration procedure (no arm_end_type branching, no
        # stroke table); it is always considered calibrated.
        return True

    def calibrate(self) -> None:
        """No-op: the metal follower requires no calibration procedure."""
        pass

    def configure(self) -> None:
        """No-op: follow gains are set in connect() from METAL_FOLLOWER_GAINS / config.gains."""
        pass

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        start = time.perf_counter()

        obs_dict: dict[str, Any] = {}

        positions = self.bus.sync_read("Present_Position")
        for motor in self._joint_motor_names:
            obs_dict[f"{motor}.pos"] = positions[motor]

        for cam_key, cam in self.cameras.items():
            start = time.perf_counter()
            obs_dict[cam_key] = cam.read_latest()
            dt_ms = (time.perf_counter() - start) * 1e3
            logger.debug(f"{self} read {cam_key}: {dt_ms:.1f}ms")

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_observation took: {dt_ms:.1f}ms")

        return obs_dict

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        goal_pos = {key.removesuffix(".pos"): val for key, val in action.items() if key.endswith(".pos")}

        # Clamp the 6 arm joints to their soft limits; the gripper is left unclamped.
        for motor_name, position in goal_pos.items():
            if motor_name in METAL_JOINT_LIMITS_DEG:
                min_limit, max_limit = METAL_JOINT_LIMITS_DEG[motor_name]
                clipped_position = max(min_limit, min(max_limit, position))
                if clipped_position != position:
                    logger.debug(f"Clipped {motor_name} from {position:.2f}° to {clipped_position:.2f}°")
                goal_pos[motor_name] = clipped_position

        # Slow initial sync: at teleop start cap each joint's per-step motion until the follower
        # has caught up to the leader, so firm follow gains don't snap it across a large gap.
        # Sync is per joint: a joint that cannot converge (parked past a soft limit, or stiction
        # above what kp * step can overcome) is released after a stall instead of capping the
        # whole arm forever; max_relative_target still bounds its speed after release.
        if self.config.startup_sync_speed_deg is not None and not self._synced:
            present_pos = self.bus.sync_read("Present_Position")
            step = self.config.startup_sync_speed_deg
            now = time.perf_counter()
            for motor_name, position in goal_pos.items():
                if motor_name in self._synced_motors:
                    continue
                present = present_pos[motor_name]
                err = position - present
                if abs(err) <= self.config.startup_sync_tolerance_deg:
                    self._synced_motors.add(motor_name)
                    continue
                last_pos, last_progress_t = self._sync_progress.get(motor_name, (present, now))
                if abs(present - last_pos) > _SYNC_PROGRESS_EPS_DEG:
                    self._sync_progress[motor_name] = (present, now)
                elif now - last_progress_t > _SYNC_STALL_RELEASE_SEC:
                    logger.warning(
                        f"{self} startup sync stalled on {motor_name} ({abs(err):.1f} deg from goal); "
                        "releasing it to full speed."
                    )
                    self._synced_motors.add(motor_name)
                    continue
                else:
                    self._sync_progress.setdefault(motor_name, (present, now))
                goal_pos[motor_name] = present + max(-step, min(step, err))
            if len(self._synced_motors) >= len(goal_pos):
                self._synced = True
                logger.info(f"{self} synced to leader; tracking at full speed.")

        # Cap goal position when too far away from present position.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        torque_ff = self.config.torque_feedforward and self._torque_ff_model is not None
        if self.config.velocity_feedforward or torque_ff:
            if self.config.velocity_feedforward:
                velocities = self._estimate_command_velocities(goal_pos)
            else:
                velocities = dict.fromkeys(goal_pos, 0.0)
            torques = self._feedforward_torques() if torque_ff else dict.fromkeys(goal_pos, 0.0)
            # Gains come from the bus's live store (sync_write("Kp"/"Kd")) so tools that
            # retune gains at runtime keep affecting the wire, same as the legacy path.
            commands = {
                motor: (*self.bus.get_gains(motor), position, velocities[motor], torques.get(motor, 0.0))
                for motor, position in goal_pos.items()
            }
            self.bus.sync_write_mit(commands)
        else:
            self.bus.sync_write("Goal_Position", goal_pos)

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
