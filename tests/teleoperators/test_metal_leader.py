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

import math
from unittest.mock import MagicMock

import pytest

from lerobot.teleoperators.metal_leader.config_metal_leader import MetalLeaderConfig
from lerobot.teleoperators.metal_leader.metal_leader import MetalLeader


@pytest.fixture
def leader():
    t = MetalLeader(MetalLeaderConfig(port="can1", gravity_hz=200))
    t.bus = MagicMock()
    t.bus.sync_read.return_value = {m: 0.0 for m in t._joint_motor_names}
    t.bus.sync_read_all_states.return_value = {
        m: {"position": 0.0, "velocity": 0.0, "torque": 0.0} for m in t._joint_motor_names
    }
    t._gravity = MagicMock()
    t._gravity.feedforward_torque.return_value = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
    return t


def test_action_features_match_seven_motors(leader):
    assert set(leader.action_features) == {f"{m}.pos" for m in leader._joint_motor_names}
    assert "gripper.pos" in leader.action_features


def test_get_action_returns_positions(leader):
    assert leader.get_action() == {f"{m}.pos": 0.0 for m in leader._joint_motor_names}


def test_gravity_tick_sends_mit_zero_kp_and_gravity_torque(leader):
    leader._gravity_tick()
    (cmds,), _ = leader.bus.sync_write_mit.call_args
    kp, kd, pos, vel, tau = cmds["joint1"]
    assert kp == 0.0 and tau == 0.1
    gkp, gkd, gpos, gvel, gtau = cmds["gripper"]
    # gripper: kp=0 (backdrivable), no GRAVITY torque, but a friction feedforward (0.06 at rest).
    assert gkp == 0.0 and gtau == pytest.approx(0.06)


def test_gripper_friction_disabled(leader):
    leader.config.gripper_friction_scale = 0.0
    leader._gravity_tick()
    (cmds,), _ = leader.bus.sync_write_mit.call_args
    assert cmds["gripper"][4] == 0.0


def test_get_action_and_gravity_tick_hold_bus_lock_during_bus_access(leader):
    """DamiaoMotorsBus has no internal locking, so MetalLeader must serialize all bus access
    (get_action vs. the background _gravity_tick) behind leader._bus_lock. Assert the lock is
    actually held while the mocked bus methods are invoked, not just that a lock attribute
    exists.
    """
    observed_locked_during_read = []
    observed_locked_during_write = []

    def fake_sync_read(*args, **kwargs):
        observed_locked_during_read.append(leader._bus_lock.locked())
        return {m: 0.0 for m in leader._joint_motor_names}

    def fake_sync_read_all_states(*args, **kwargs):
        observed_locked_during_read.append(leader._bus_lock.locked())
        return {m: {"position": 0.0, "velocity": 0.0, "torque": 0.0} for m in leader._joint_motor_names}

    def fake_sync_write_mit(*args, **kwargs):
        observed_locked_during_write.append(leader._bus_lock.locked())

    leader.bus.sync_read.side_effect = fake_sync_read
    leader.bus.sync_read_all_states.side_effect = fake_sync_read_all_states
    leader.bus.sync_write_mit.side_effect = fake_sync_write_mit

    leader.get_action()  # uses sync_read
    assert observed_locked_during_read == [True]

    observed_locked_during_read.clear()
    leader._gravity_tick()  # uses sync_read_all_states
    assert observed_locked_during_read == [True]
    assert observed_locked_during_write == [True]

    # The lock must be released again after each call completes.
    assert not leader._bus_lock.locked()


def test_friction_feedforward_uses_measured_velocity(leader):
    leader.bus.sync_read_all_states.return_value = {
        m: {"position": 0.0, "velocity": 30.0, "torque": 0.0} for m in leader._joint_motor_names
    }  # 30 deg/s, above the deadzone
    leader.config.friction_scale = 0.5
    leader._gravity_tick()
    calls = leader._gravity.feedforward_torque.call_args_list
    assert len(calls) == 2  # gravity-only pass, then a velocity pass for friction/coriolis
    (_, dq_grav), _ = calls[0]
    (_, dq_fric), _ = calls[1]
    assert dq_grav == [0.0] * 6
    assert dq_fric[0] == pytest.approx(math.radians(30.0))  # deg/s -> rad/s


def test_friction_disabled_skips_velocity_pass(leader):
    leader.config.friction_scale = 0.0
    leader._gravity_tick()
    assert leader._gravity.feedforward_torque.call_count == 1  # gravity only, no velocity pass


def test_leader_kd_per_joint_dict():
    t = MetalLeader(MetalLeaderConfig(port="can1", leader_kd={"joint1": 0.5, "joint2": 0.2}))
    t.bus = MagicMock()
    t.bus.sync_read_all_states.return_value = {
        m: {"position": 0.0, "velocity": 0.0, "torque": 0.0} for m in t._joint_motor_names
    }
    t._gravity = MagicMock()
    t._gravity.feedforward_torque.return_value = [0.0] * 6
    t._gravity_tick()
    (cmds,), _ = t.bus.sync_write_mit.call_args
    assert cmds["joint1"][1] == 0.5
    assert cmds["joint2"][1] == 0.2
    assert cmds["joint3"][1] == 0.0  # absent from dict -> 0


def test_friction_scale_per_joint_dict():
    t = MetalLeader(MetalLeaderConfig(port="can1", leader_kd=0.0, friction_scale={"joint1": 2.0}))
    t.bus = MagicMock()
    t.bus.sync_read_all_states.return_value = {
        m: {"position": 0.0, "velocity": 30.0, "torque": 0.0} for m in t._joint_motor_names
    }
    t._gravity = MagicMock()
    t._gravity.feedforward_torque.side_effect = [[1.0] * 6, [2.0] * 6]  # gravity, then full
    t._gravity_tick()
    (cmds,), _ = t.bus.sync_write_mit.call_args
    assert cmds["joint1"][4] == pytest.approx(3.0)  # 1.0 + 2.0*(2.0-1.0)
    assert cmds["joint2"][4] == pytest.approx(1.0)  # absent -> scale 0 -> gravity only


def test_factory_builds_metal_leader():
    from lerobot.teleoperators.utils import make_teleoperator_from_config
    from lerobot.teleoperators.metal_leader.config_metal_leader import MetalLeaderConfig

    t = make_teleoperator_from_config(MetalLeaderConfig(port="can1"))
    assert t.name == "metal_leader"
    assert type(t).__name__ == "MetalLeader"
