#!/usr/bin/env python

# Copyright 2026 The HuggingFace Inc. team. All rights reserved.
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

from lerobot.cameras.opencv import OpenCVCameraConfig
from lerobot.robots.bi_metal_follower import BiMetalFollower, BiMetalFollowerConfig
from lerobot.robots.metal_follower.config_metal_follower import MetalFollowerConfig


def _cam(path: str) -> OpenCVCameraConfig:
    return OpenCVCameraConfig(index_or_path=path, width=640, height=480, fps=30)


def _make_config(**overrides) -> BiMetalFollowerConfig:
    kwargs = {
        "left_arm_config": MetalFollowerConfig(
            port="can1", startup_sync_speed_deg=None, velocity_feedforward=False
        ),
        "right_arm_config": MetalFollowerConfig(
            port="can0", startup_sync_speed_deg=None, velocity_feedforward=False
        ),
    }
    kwargs.update(overrides)
    return BiMetalFollowerConfig(**kwargs)


@pytest.fixture
def follower():
    r = BiMetalFollower(_make_config())
    for arm in (r.left_arm, r.right_arm):
        arm.bus = MagicMock()
        arm.bus.sync_read.return_value = dict.fromkeys(arm._joint_motor_names, 0.0)
    return r


def test_motor_features_are_prefixed(follower):
    motor_keys = list(follower.action_features)
    assert len(motor_keys) == 14
    assert motor_keys[:7] == [f"left_{m}.pos" for m in follower.left_arm._joint_motor_names]
    assert motor_keys[7:] == [f"right_{m}.pos" for m in follower.right_arm._joint_motor_names]
    assert set(follower.action_features).issubset(follower.observation_features)


def test_camera_features_top_level_unprefixed_per_arm_prefixed():
    config = _make_config(cameras={"top": _cam("/dev/video0")})
    config.left_arm_config.cameras = {"wrist": _cam("/dev/video2")}
    config.right_arm_config.cameras = {"wrist": _cam("/dev/video4")}
    r = BiMetalFollower(config)

    cam_features = {k: v for k, v in r.observation_features.items() if k not in r.action_features}
    assert set(cam_features) == {"top", "left_wrist", "right_wrist"}
    assert cam_features["top"] == (480, 640, 3)


def test_top_level_camera_name_collision_raises():
    config = _make_config(cameras={"wrist": _cam("/dev/video0")})
    config.left_arm_config.cameras = {"wrist": _cam("/dev/video2")}
    with pytest.raises(ValueError, match="collide"):
        BiMetalFollower(config)


def test_send_action_routes_by_prefix(follower):
    action = {
        **{f"left_{m}.pos": 10.0 for m in follower.left_arm._joint_motor_names},
        **{f"right_{m}.pos": 20.0 for m in follower.right_arm._joint_motor_names},
    }
    sent = follower.send_action(action)

    left_goals = follower.left_arm.bus.sync_write.call_args.args[1]
    right_goals = follower.right_arm.bus.sync_write.call_args.args[1]
    assert left_goals["joint1"] == 10.0
    assert right_goals["joint1"] == 20.0
    assert sent["left_joint1.pos"] == 10.0
    assert sent["right_joint1.pos"] == 20.0
    assert set(sent) == set(action)


def test_get_observation_prefixes_arm_keys(follower):
    follower.left_arm.bus.sync_read.return_value = dict.fromkeys(follower.left_arm._joint_motor_names, 1.0)
    follower.right_arm.bus.sync_read.return_value = dict.fromkeys(follower.right_arm._joint_motor_names, 2.0)
    obs = follower.get_observation()
    assert obs["left_joint1.pos"] == 1.0
    assert obs["right_joint1.pos"] == 2.0
    assert len(obs) == 14


def test_cli_parser_builds_without_recursion():
    # Building the argparse tree expands every registered RobotConfig choice; a per-arm
    # field typed as a registered choice subclass makes that expansion self-referential
    # (bi -> arm -> choice registry -> bi -> ...) and blows the stack.
    import dataclasses

    import draccus.argparsing

    from lerobot.robots import RobotConfig

    @dataclasses.dataclass
    class _Cfg:
        robot: RobotConfig | None = None

    draccus.argparsing.ArgumentParser(_Cfg)


def test_factory_builds_bi_metal_follower():
    from lerobot.robots.utils import make_robot_from_config

    r = make_robot_from_config(_make_config())
    assert r.name == "bi_metal_follower"
    assert type(r).__name__ == "BiMetalFollower"
