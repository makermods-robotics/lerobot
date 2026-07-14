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
import statistics
import time
from collections import deque
from functools import cached_property
from pathlib import Path
from typing import Any

from lerobot.cameras import make_cameras_from_configs
from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.metal import METAL_FOLLOWER_GAINS, METAL_JOINT_LIMITS_DEG, METAL_MOTOR_CONFIG
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

        # Acceleration-limiter state (only used when config.max_relative_accel_deg is set):
        # last commanded goal and last per-step motion, per motor. Reset on every connect().
        self._prev_goal: dict[str, float] = {}
        self._prev_step: dict[str, float] = {}

        # Median-filter state (only used when config.median_filter_window > 1): recent raw actions
        # per motor. Reset on every connect().
        self._goal_hist: dict[str, deque] = {}

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
        self._synced = False  # re-arm the slow initial sync on every connect
        self._prev_goal.clear()  # re-arm the acceleration limiter
        self._prev_step.clear()
        self._goal_hist.clear()  # re-arm the median filter

        # Set firm follow gains (bus default kp=10 is far too soft to hold the arm against
        # gravity → the follower sags). Uses vendor follow_mit_kp/kd unless overridden.
        gains = self.config.gains or METAL_FOLLOWER_GAINS
        gains = {m: kpkd for m, kpkd in gains.items() if m in self._joint_motor_names}
        self.bus.sync_write("Kp", {m: kp for m, (kp, kd) in gains.items()})
        self.bus.sync_write("Kd", {m: kd for m, (kp, kd) in gains.items()})

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

        # Median filter (on the raw action, before anything else): a single-frame spike in the
        # policy output is discarded by the median while real motion passes through. Best defense
        # against per-frame jitter. Disabled when window <= 1. Does not touch kp/kd.
        window = self.config.median_filter_window
        if window and window > 1:
            for motor_name, position in goal_pos.items():
                buf = self._goal_hist.setdefault(motor_name, deque(maxlen=window))
                buf.append(position)
                goal_pos[motor_name] = statistics.median(buf)

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
        if self.config.startup_sync_speed_deg is not None and not self._synced:
            present_pos = self.bus.sync_read("Present_Position")
            step = self.config.startup_sync_speed_deg
            max_err = 0.0
            for motor_name, position in goal_pos.items():
                err = position - present_pos[motor_name]
                max_err = max(max_err, abs(err))
                goal_pos[motor_name] = present_pos[motor_name] + max(-step, min(step, err))
            if max_err <= self.config.startup_sync_tolerance_deg:
                self._synced = True
                logger.info(f"{self} synced to leader; tracking at full speed.")

        # Cap goal position when too far away from present position.
        if self.config.max_relative_target is not None:
            present_pos = self.bus.sync_read("Present_Position")
            goal_present_pos = {key: (g_pos, present_pos[key]) for key, g_pos in goal_pos.items()}
            goal_pos = ensure_safe_goal_position(goal_present_pos, self.config.max_relative_target)

        # Acceleration (jerk) limit: cap how much each joint's per-step motion can change from the
        # previous step, so the arm can't suddenly accelerate/decelerate. Operates on the commanded
        # goal trajectory and does not touch kp/kd. Smooths starts/stops and policy-output spikes.
        accel = self.config.max_relative_accel_deg
        if accel is not None:
            for motor_name, position in goal_pos.items():
                prev_goal = self._prev_goal.get(motor_name, position)
                prev_step = self._prev_step.get(motor_name, 0.0)
                desired_step = position - prev_goal
                limited_step = max(prev_step - accel, min(prev_step + accel, desired_step))
                goal_pos[motor_name] = prev_goal + limited_step
                self._prev_step[motor_name] = limited_step
            self._prev_goal.update(goal_pos)

        self.bus.sync_write("Goal_Position", goal_pos)

        return {f"{motor}.pos": val for motor, val in goal_pos.items()}

    @check_if_not_connected
    def disconnect(self) -> None:
        self.bus.disconnect(self.config.disable_torque_on_disconnect)

        for cam in self.cameras.values():
            cam.disconnect()

        logger.info(f"{self} disconnected.")
