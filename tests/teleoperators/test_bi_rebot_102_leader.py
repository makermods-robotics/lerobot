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

from unittest.mock import MagicMock, patch

import pytest

from lerobot.teleoperators.bi_rebot_102_leader import BiRebot102Leader, BiRebot102LeaderConfig
from lerobot.teleoperators.rebot_102_leader import RebotArm102LeaderConfig

_MODULE = "lerobot.teleoperators.rebot_102_leader.rebot_102_leader"


def _make_bus_mock() -> MagicMock:
    bus = MagicMock(name="FashionStarServoMock")
    bus.ping.return_value = True

    def _sync_monitor(ids):
        monitors = {}
        for servo_id in ids:
            monitor = MagicMock()
            monitor.angle_deg = 5.0
            monitors[servo_id] = monitor
        return monitors

    bus.sync_monitor.side_effect = _sync_monitor
    return bus


def _make_config() -> BiRebot102LeaderConfig:
    return BiRebot102LeaderConfig(
        left_arm_config=RebotArm102LeaderConfig(port="/dev/star_left"),
        right_arm_config=RebotArm102LeaderConfig(port="/dev/star_right"),
    )


@pytest.fixture
def leader():
    with (
        patch(f"{_MODULE}.require_package", lambda *a, **kw: None),
        patch(f"{_MODULE}.FashionStarServo", side_effect=lambda *a, **kw: _make_bus_mock()),
    ):
        teleop = BiRebot102Leader(_make_config())
        teleop.connect(calibrate=False)
        yield teleop
        if teleop.is_connected:
            teleop.disconnect()


def test_action_features_prefixed(leader):
    keys = list(leader.action_features)
    assert len(keys) == 14
    assert all(k.startswith("left_") for k in keys[:7])
    assert all(k.startswith("right_") for k in keys[7:])


def test_get_action_prefixes_both_arms(leader):
    action = leader.get_action()
    assert len(action) == 14
    # shoulder_pan direction is -1, so a +5deg raw reading flips to -5deg on both arms.
    assert action["left_shoulder_pan.pos"] == pytest.approx(-5.0)
    assert action["right_shoulder_pan.pos"] == pytest.approx(-5.0)


def test_run_both_executes_concurrently(leader):
    """Left and right reads overlap in time rather than running back-to-back."""
    import threading

    barrier = threading.Barrier(2, timeout=2.0)

    def left():
        barrier.wait()  # only returns if the right call is also in-flight
        return "L"

    def right():
        barrier.wait()
        return "R"

    # A serial _run_both would deadlock here: the first barrier.wait() blocks forever
    # because the second call never starts, tripping the timeout.
    assert leader._run_both(left, right) == ("L", "R")


def test_run_both_propagates_exception(leader):
    def boom():
        raise RuntimeError("leader failure")

    with pytest.raises(RuntimeError, match="leader failure"):
        leader._run_both(boom, lambda: "ok")


def test_send_feedback_not_implemented(leader):
    with pytest.raises(NotImplementedError):
        leader.send_feedback({})
