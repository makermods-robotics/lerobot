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
    r = MetalFollower(MetalFollowerConfig(port="can0"))
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
