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

from lerobot.robots.metal_follower.config_metal_follower import MetalFollowerConfig
from lerobot.robots.metal_follower.metal_follower import MetalFollower


@pytest.fixture
def follower():
    # startup slow-sync off so these tests exercise soft limits / gains directly (a dedicated
    # test covers the sync behaviour).
    r = MetalFollower(MetalFollowerConfig(port="can0", startup_sync_speed_deg=None))
    r.bus = MagicMock()
    r.bus.sync_read.return_value = {m: 0.0 for m in r._joint_motor_names}
    return r


def test_has_all_seven_motors(follower):
    assert follower._joint_motor_names == [
        "joint1",
        "joint2",
        "joint3",
        "joint4",
        "joint5",
        "joint6",
        "gripper",
    ]


def test_action_features_include_all_motors(follower):
    assert "joint1.pos" in follower.action_features
    assert "gripper.pos" in follower.action_features


def test_send_action_writes_goal_and_clamps(follower):
    action = {f"{m}.pos": 999.0 for m in follower._joint_motor_names}
    out = follower.send_action(action)
    assert follower.bus.sync_write.called
    assert out["joint1.pos"] == 160.0  # clamped to joint1 upper soft limit


def test_factory_builds_metal_follower():
    from lerobot.robots.utils import make_robot_from_config
    from lerobot.robots.metal_follower.config_metal_follower import MetalFollowerConfig

    r = make_robot_from_config(MetalFollowerConfig(port="can0"))
    assert r.name == "metal_follower"
    assert type(r).__name__ == "MetalFollower"


def test_startup_slow_sync_clamps_then_syncs():
    r = MetalFollower(MetalFollowerConfig(port="can0", startup_sync_speed_deg=3.0, startup_sync_tolerance_deg=3.0))
    r.bus = MagicMock()
    r.bus.sync_read.return_value = {"joint1": 0.0}
    out = r.send_action({"joint1.pos": 50.0})
    assert out["joint1.pos"] == 3.0  # clamped to +3 deg/step from present 0
    assert r._synced is False
    r.bus.sync_read.return_value = {"joint1": 48.0}
    r.send_action({"joint1.pos": 50.0})  # err 2 <= tolerance 3 -> synced
    assert r._synced is True


def test_connect_sets_follow_gains(follower):
    from lerobot.motors.metal.constants import METAL_FOLLOWER_GAINS

    follower.cameras = {}
    follower.bus.is_connected = False  # so @check_if_already_connected doesn't trip
    follower.connect(calibrate=False)
    kp_call = {m: kp for m, (kp, kd) in METAL_FOLLOWER_GAINS.items()}
    kd_call = {m: kd for m, (kp, kd) in METAL_FOLLOWER_GAINS.items()}
    follower.bus.sync_write.assert_any_call("Kp", kp_call)
    follower.bus.sync_write.assert_any_call("Kd", kd_call)
