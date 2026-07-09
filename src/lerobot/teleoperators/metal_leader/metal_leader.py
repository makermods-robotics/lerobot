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
import threading
import time
from math import radians
from pathlib import Path
from typing import Any

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.damiao import DamiaoMotorsBus
from lerobot.motors.metal import METAL_GRIPPER_NAME, METAL_JOINT_NAMES, METAL_MOTOR_CONFIG
from lerobot.motors.metal.gravity import MetalGravityModel
from lerobot.types import RobotAction
from lerobot.utils.decorators import check_if_already_connected, check_if_not_connected

from ..teleoperator import Teleoperator
from .config_metal_leader import MetalLeaderConfig

logger = logging.getLogger(__name__)

# Bundled URDF used by default when `config.urdf_path` is left empty. The gripper is permanent
# on the metal arm, so this is always the "with gripper" variant (no arm_end_type branching).
DEFAULT_URDF_PATH = str(
    Path(__file__).resolve().parents[2] / "motors" / "metal" / "urdf" / "metal_with_gripper.urdf"
)


class MetalLeader(Teleoperator):
    """
    Metal arm leader/teleoperator: 6 joints + a permanent gripper, all driven as Damiao motors over
    classic CAN (`use_can_fd=False`) via the stock `DamiaoMotorsBus`.

    Unlike torque-disabled leaders, the human moves this arm while a background thread continuously
    streams Pinocchio-computed gravity-compensation MIT torque (`kp=0`, feedforward torque from the
    URDF dynamics model), so the arm feels weightless. The gripper is left fully backdrivable
    (`kp=0`, `torque=0`) so it can be squeezed freely; the resulting raw gripper angle is read back
    by `get_action` and drives the follower gripper 1:1.
    """

    config_class = MetalLeaderConfig
    name = "metal_leader"

    def __init__(self, config: MetalLeaderConfig):
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

        # Built lazily on connect() (Pinocchio is heavy and lazy-imported by MetalGravityModel
        # itself; we don't want it loaded just from importing this module).
        self._gravity: MetalGravityModel | None = None
        self._gravity_thread: threading.Thread | None = None
        self._gravity_stop_event = threading.Event()

        # DamiaoMotorsBus has no internal locking: both the background gravity-compensation
        # thread (_gravity_tick) and the main-loop get_action() send a CAN request then poll the
        # shared canbus.recv() and mutate a shared state cache. All bus access from this
        # teleoperator must be serialized behind this lock to avoid racing on the CAN socket.
        self._bus_lock = threading.Lock()

    @property
    def action_features(self) -> dict[str, type]:
        return {f"{motor}.pos": float for motor in self._joint_motor_names}

    @property
    def feedback_features(self) -> dict[str, type]:
        return {}

    @property
    def is_connected(self) -> bool:
        return self.bus.is_connected

    @check_if_already_connected
    def connect(self, calibrate: bool = True) -> None:
        logger.info(f"Connecting arm on {self.config.port}...")
        self.bus.connect()

        # Build the gravity model BEFORE enabling torque: if the URDF / Pinocchio load fails,
        # the arm must not be left powered with no gravity-compensation thread running.
        try:
            self._gravity = MetalGravityModel(self.config.urdf_path)
        except Exception:
            self.bus.disconnect(disable_torque=False)
            raise
        self.bus.enable_torque()

        self._gravity_stop_event.clear()
        self._gravity_thread = threading.Thread(target=self._gravity_loop, daemon=True)
        self._gravity_thread.start()

        logger.info(f"{self} connected.")

    @property
    def is_calibrated(self) -> bool:
        # The metal arm has no separate calibration procedure (no arm_end_type branching, no
        # stroke table); it is always considered calibrated.
        return True

    def calibrate(self) -> None:
        """No-op: the metal leader requires no calibration procedure."""
        pass

    def configure(self) -> None:
        """No-op: gravity-compensation gains (kp=0, kd=config.leader_kd) are applied per-tick."""
        pass

    def _gravity_loop(self) -> None:
        period = 1.0 / self.config.gravity_hz
        while not self._gravity_stop_event.is_set():
            start = time.perf_counter()
            self._gravity_tick()
            elapsed = time.perf_counter() - start
            sleep_time = period - elapsed
            if sleep_time > 0:
                self._gravity_stop_event.wait(sleep_time)

    def _gravity_tick(self) -> None:
        """One gravity-compensation control cycle: read positions, compute gravity feedforward
        torque via Pinocchio, and stream zero-kp MIT commands (backdrivable gripper) to the bus.

        Factored out of `_gravity_loop` so tests can call it directly without a running thread.
        Wrapped in try/except so a single bad tick (e.g. a transient CAN read failure) never kills
        the background thread.
        """
        try:
            with self._bus_lock:
                states = self.bus.sync_read_all_states()  # position (deg), velocity (deg/s), torque
                present = {m: states[m]["position"] for m in self._joint_motor_names}
                q_rad = [radians(states[m]["position"]) for m in METAL_JOINT_NAMES]
                tau = self._gravity.feedforward_torque(q_rad, [0.0] * len(METAL_JOINT_NAMES))
                # Resolve friction scale per joint (float -> all; dict -> per-joint, missing -> 0).
                fs_cfg = self.config.friction_scale
                if isinstance(fs_cfg, dict):
                    scales = [float(fs_cfg.get(m, 0.0)) for m in METAL_JOINT_NAMES]
                else:
                    scales = [float(fs_cfg)] * len(METAL_JOINT_NAMES)
                if self.config.use_velocity_feedforward and any(s > 0.0 for s in scales):
                    # Feed measured velocity to activate friction/coriolis comp, scaled per joint so
                    # the arm feels transparent without running away. Deadzone rejects noise at rest.
                    dz = self.config.velocity_deadzone_rad_s
                    dq_rad = []
                    for m in METAL_JOINT_NAMES:
                        v = radians(states[m]["velocity"])
                        dq_rad.append(0.0 if abs(v) < dz else v)
                    tau_full = self._gravity.feedforward_torque(q_rad, dq_rad)
                    tau = [tau[i] + scales[i] * (tau_full[i] - tau[i]) for i in range(len(tau))]

                # Resolve kd per motor (float -> all joints; dict -> per-joint, missing -> 0).
                kd_cfg = self.config.leader_kd
                if isinstance(kd_cfg, dict):
                    def kd_of(m):
                        return float(kd_cfg.get(m, 0.0))
                else:
                    def kd_of(m):
                        return float(kd_cfg)

                commands: dict[str, tuple[float, float, float, float, float]] = {}
                for i, motor in enumerate(METAL_JOINT_NAMES):
                    commands[motor] = (0.0, kd_of(motor), present[motor], 0.0, tau[i])
                # Gripper: backdrivable, no gravity torque, so the human can squeeze it freely.
                commands[METAL_GRIPPER_NAME] = (
                    0.0,
                    kd_of(METAL_GRIPPER_NAME),
                    present[METAL_GRIPPER_NAME],
                    0.0,
                    0.0,
                )

                self.bus.sync_write_mit(commands)
        except Exception:
            logger.exception("Gravity-compensation tick failed; continuing.")

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        """
        Retrieve the leader's current joint positions (all 7 motors, degrees). The gripper value is
        raw motor degrees, matching `MetalFollower.action_features`, so squeezing the leader gripper
        drives the follower gripper 1:1.
        """
        start = time.perf_counter()

        with self._bus_lock:
            positions = self.bus.sync_read("Present_Position")
        action_dict: dict[str, Any] = {f"{motor}.pos": positions[motor] for motor in self._joint_motor_names}

        dt_ms = (time.perf_counter() - start) * 1e3
        logger.debug(f"{self} get_action took: {dt_ms:.1f}ms")

        return action_dict

    def send_feedback(self, feedback: dict[str, Any]) -> None:
        raise NotImplementedError("Feedback is not implemented for the metal leader.")

    @check_if_not_connected
    def disconnect(self) -> None:
        self._gravity_stop_event.set()
        if self._gravity_thread is not None:
            self._gravity_thread.join(timeout=1.0)
            if self._gravity_thread.is_alive():
                logger.warning(
                    f"{self} gravity-compensation thread did not stop within timeout; "
                    "it may still be running."
                )
            self._gravity_thread = None

        # Keep torque enabled so the arm holds its last commanded position instead of free-falling.
        self.bus.disconnect(disable_torque=False)

        logger.info(f"{self} disconnected.")
