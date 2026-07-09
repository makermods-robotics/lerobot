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

from unittest.mock import MagicMock

import pytest

from lerobot.teleoperators.metal_leader.config_metal_leader import MetalLeaderConfig
from lerobot.teleoperators.metal_leader.metal_leader import MetalLeader


@pytest.fixture
def leader():
    t = MetalLeader(MetalLeaderConfig(port="can1", gravity_hz=200))
    t.bus = MagicMock()
    t.bus.sync_read.return_value = {m: 0.0 for m in t._joint_motor_names}
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
    assert gkp == 0.0 and gtau == 0.0  # gripper backdrivable, no gravity torque


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

    def fake_sync_write_mit(*args, **kwargs):
        observed_locked_during_write.append(leader._bus_lock.locked())

    leader.bus.sync_read.side_effect = fake_sync_read
    leader.bus.sync_write_mit.side_effect = fake_sync_write_mit

    leader.get_action()
    assert observed_locked_during_read == [True]

    observed_locked_during_read.clear()
    leader._gravity_tick()
    assert observed_locked_during_read == [True]
    assert observed_locked_during_write == [True]

    # The lock must be released again after each call completes.
    assert not leader._bus_lock.locked()


def test_factory_builds_metal_leader():
    from lerobot.teleoperators.utils import make_teleoperator_from_config
    from lerobot.teleoperators.metal_leader.config_metal_leader import MetalLeaderConfig

    t = make_teleoperator_from_config(MetalLeaderConfig(port="can1"))
    assert t.name == "metal_leader"
    assert type(t).__name__ == "MetalLeader"
